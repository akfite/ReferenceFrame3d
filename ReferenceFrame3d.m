classdef ReferenceFrame3d < matlab.mixin.Copyable ...
        & matlab.mixin.CustomDisplay ...
        & matlab.mixin.SetGet
    
    properties
        name(1,1) string = missing % optional text label to be used with plot()
        T(4,4) double = eye(4) % homogeneous transform (rotation & translation)
    end

    properties (Dependent)
        R(3,3) double % rotation submatrix
        x(3,1) double % x basis
        y(3,1) double % y basis
        z(3,1) double % z basis
        origin(3,1) double % translation (origin of reference frame)
    end

    % graphics
    properties (Access = protected, Transient)
        h_transform matlab.graphics.primitive.Transform
        h_plot_group matlab.graphics.primitive.Group
    end

    %% Construct/Update
    methods
        function obj = ReferenceFrame3d(rotation, origin, opts)
            %REFERENCEFRAME3D Constructor.
            arguments
                rotation = eye(3) % supports various types
                origin(1,3) double {mustBeReal, mustBeFinite} = [0; 0; 0]
                opts.Name(1,1) string = missing
            end

            sz = size(rotation);

            if isscalar(rotation)
                switch string(class(rotation))
                    case "ReferenceFrame3d"
                        T = rotation.T;
                        obj.name = rotation.name;
                    case "quaternion"
                        T = rotmat(rotation,"point");
                        T(4,4) = 1; % 3x3 -> 4x4
                        T(1:3,4) = origin;
                    case "se3"
                        T = tform(rotation);
                    case "so3"
                        T = tform(rotation);
                        T(1:3,4) = origin;
                    otherwise
                        error('Unsupported scalar type %s', class(rotation));
                end
            elseif isequal(sz, [4 4]) % homogeneous transform (4x4 matrix)
                T = rotation;
            elseif isequal(sz, [3 3]) % DCM + origin
                rotation(4,4) = 1;
                rotation(1:3,4) = origin;
                T = rotation;
            else
                error('Expected rotation to be a 3x3 or 4x4 matrix.');
            end

            obj.T = T; % triggers set.T()
            obj.name = opts.Name;
        end
    end

    methods (Static)
        function obj = from_point_normal(point, normal, opts)
            %FROM_POINT_NORMAL Creates a new frame to represent a plane.
            arguments
                point(1,3) double {mustBeReal, mustBeFinite}
                normal(1,3) double {mustBeReal, mustBeFinite}
                opts.Name(1,1) string = "normal"
            end

            % pick any arbitrary orthogonal x and y vectors
            z_basis = normal / norm(normal);
            
            if abs(z_basis(1)) < 0.9  % if z is not too close to x-axis
                whatever = [1; 0; 0];
            else  % if z is close to x-axis, use y-axis instead
                whatever = [0; 1; 0];
            end
            
            x_basis = cross(whatever, z_basis);
            x_basis = x_basis / norm(x_basis);
            
            y_basis = cross(z_basis, x_basis);
            y_basis = y_basis / norm(y_basis);

            % create the transform
            T = [x_basis; y_basis; z_basis]';
            T(4,4) = 1;
            T(1:3,4) = point;
            obj = ReferenceFrame3d(T, "Name", opts.Name);
        end

        function obj = from_coplanar_vectors(v1, v2, origin, opts)
            %FROM_COPLANAR_VECTORS Create a frame using a coplanar vector pair.
            arguments
                v1(1,3) double {mustBeReal, mustBeFinite}
                v2(1,3) double {mustBeReal, mustBeFinite}
                origin(1,3) double = [0 0 0]
                opts.Name(1,1) string = "coplanar"
            end

            x_basis = v1;
            z_basis = cross(v1, v2);
            y_basis = cross(z_basis, x_basis);

            dcm = [x_basis(:) y_basis(:) z_basis(:)];
            dcm = dcm ./ vecnorm(dcm, 2, 1);

            obj = ReferenceFrame3d(dcm, origin, "Name", opts.Name);
        end

        function obj = from_euler(angles, origin, opts)
            %FROM_EULER Create a transform from zyx euler sequence.
            arguments
                angles(1,3) double {mustBeReal, mustBeFinite}
                origin(1,3) double {mustBeReal, mustBeFinite} = [0 0 0]
                opts.Sequence(1,3) char = 'zyx'
                opts.Units(1,1) string = "deg"
                opts.Name(1,1) string = missing
            end

            obj = ReferenceFrame3d(eye(3), origin, "Name", opts.Name);
            obj.rotate_euler(angles, ...
                "Units", opts.Units, ...
                "Sequence", opts.Sequence);
        end

        function obj = from_campos(ax, opts)
            %FROM_CAMPOS Create a transform for the current camera perspective.
            arguments
                ax(1,1) matlab.graphics.axis.Axes = gca
                opts.Name(1,1) string = "campos"
            end

            obj = ReferenceFrame3d.from_view_axis(...
                campos(ax), ...
                camtarget(ax), ...
                camup(ax), ...
                "Name", opts.Name);
        end

        function obj = from_view_axis(observer, target, up, opts)
            %FROM_VIEW_AXIS Create a transform for an observer's perspective.
            arguments
                observer(1,3) double {mustBeReal, mustBeFinite} % position
                target(1,3) double {mustBeReal, mustBeFinite} % position
                up(1,3) double {mustBeReal, mustBeFinite} % up at observer
                opts.Name(1,1) string = "view axis"
            end

            z = target - observer; % viewing axis
            x = cross(z, up); % right
            y = cross(z, x); % down

            dcm = [x(:) y(:) z(:)];
            dcm = dcm ./ vecnorm(dcm, 2, 1); % make unit vectors

            obj = ReferenceFrame3d(dcm, observer, "Name", opts.Name);
        end

        function obj = ecef2ned(lla, opts)
            %ECEF2NED Local-level North-East-Down frame w.r.t. ECEF.
            arguments
                lla(1,3) double {mustBeReal, mustBeFinite} % alt w.r.t. the WGS84 ellipsoid, meters
                opts.Units(1,1) string = "deg"
                opts.Name(1,1) string = "NED"
            end

            assert(contains(opts.Units, ["deg","degrees","rad","radians"]), ...
                'Unknown angle unit (expected "deg", "degrees", "rad", "radians"; got "%s")', ...
                opts.Units);

            lat = lla(:,1);
            lon = lla(:,2);
            alt = lla(:,3);

            if contains(opts.Units, "deg")
                lat = lat * pi/180;
                lon = lon * pi/180;
            end

            slat = sin(lat);
            clat = cos(lat);
            slon = sin(lon);
            clon = cos(lon);
            
            C_E2L(3,3) = -slat; % pre-allocates matrix
            C_E2L(1,1) = -slat * clon;
            C_E2L(1,2) = -slon;
            C_E2L(1,3) = -clat * clon;
            C_E2L(2,1) = -slat * slon;
            C_E2L(2,2) = clon;
            C_E2L(2,3) = -clat * slon;
            C_E2L(3,1) = clat;

            % convert geodetic position to ECEF to set the origin
            pos_ecef = local_lla2ecef();
            obj = ReferenceFrame3d(C_E2L, pos_ecef, "Name", opts.Name);

            function pos_ecef = local_lla2ecef()
                %LOCAL_LLA2ECEF Helper function to convert geodetic coordinates to ECEF.
                a = 6378137.0;          % WGS84 semi-major axis (equatorial radius) in meters
                f = 1/298.257223563;    % flattening
                b = a*(1-f);            % semi-minor axis (polar radius)
                e2 = 1 - (b^2)/(a^2);   % square of eccentricity
                
                % calculate radius of curvature in the prime vertical
                N = a / sqrt(1 - e2 * slat^2);
                
                % calculate ECEF coordinates
                x = (N + alt) * clat * clon;
                y = (N + alt) * clat * slon;
                z = (N * (1 - e2) + alt) * slat;
                
                pos_ecef = [x y z];
            end
        end

        function obj = ecef2enu(lla, opts)
            %ECEF2ENU Local-level East-North-Up frame w.r.t. ECEF.
            arguments
                lla(1,3) double % alt w.r.t. the WGS84 ellipsoid, meters
                opts.Units(1,1) string = "deg"
                opts.Name(1,1) string = "ENU"
            end

            obj = ReferenceFrame3d.ecef2ned(lla, "Name", opts.Name, "Units", opts.Units);
            obj.rotate_dcm(...
                [ ...
                    0 1 0
                    1 0 0
                    0 0 -1
                ]);
        end

        function validate_transform(T)
            %VALIDATE_TRANSFORM Check that the transform makes sense as configured.
            validateattributes(T, ...
                {'single','double'}, ...
                {'real','finite','2d','ncols',4,'nrows',4});
            tol = 1e-10;

            R = T(1:3,1:3); %#ok<*PROP>
            assert(abs(det(R) - 1) < tol, ...
                'Expected determinant of 3x3 submatrix R to be 1; instead got %f', det(R));

            RRt = R*R';
            orthogonality_error = norm(RRt - eye(3),'fro');
            assert(orthogonality_error < tol, ...
                'Orthogonality is outside tolerance (R*R'' is not eye(3); %f > %f)', ...
                orthogonality_error, tol)

            assert(isequal(T(4,:), [0 0 0 1]), ...
                'The bottom row of the transform must be [0 0 0 1]');
        end
    end

    %% Math & Utility
    methods (Sealed)
        function varargout = local2base(frames, varargin)
            %LOCAL2BASE Transform a vector from the local to base frame (translate & rot)
            nargoutchk(0,3);

            switch nargin
                case 2 % obj, [x y z]
                    local_vec = varargin{1};
                    orig_sz = [size(local_vec,1) 1];
                case 4 % obj, x, y, z
                    orig_sz = size(varargin{1});
                    local_vec = [varargin{1}(:), varargin{2}(:), varargin{3}(:)];
                otherwise
                    error('Expected input vector as an Nx3 or three Nx1 arguments.');
            end

            if size(local_vec,2) == 3
                local_vec(:,4) = 1; % to homogeneous coordinates
            end

            if isscalar(frames)
                T = frames.T;
            else
                T = compose(frames).as_transform();
            end

            % create a dimension deleter (delete 4th dim of output)
            H = [
                1 0 0 0
                0 1 0 0
                0 0 1 0
                ];

            base_vec = H * T * local_vec'; % was Nx4; now 3xN

            if nargout <= 1
                varargout{1} = base_vec'; % back to Nx3
            else
                for i = nargout:-1:1
                    varargout{i} = reshape(base_vec(i,:), orig_sz);
                end
            end
        end

        function varargout = base2local(frames, varargin)
            %BASE2LOCAL Transform a vector from the base to local frame (translate & rot)
            nargoutchk(0,3);

            switch nargin
                case 2 % obj, [x y z]
                    base_vec = varargin{1};
                    orig_sz = [size(base_vec,1) 1];
                case 4 % obj, x, y, z
                    orig_sz = size(varargin{1});
                    base_vec = [varargin{1}(:), varargin{2}(:), varargin{3}(:)];
                otherwise
                    error('Expected input vector as an Nx3 or three Nx1 arguments.');
            end

            if size(base_vec,2) == 3
                base_vec(:,4) = 1; % to homogeneous coordinates
            end

            if isscalar(frames)
                T = inv(frames).as_transform();
            else
                T = inv(compose(frames)).as_transform();
            end

            % create a dimension deleter (delete 4th dim of output)
            H = [
                1 0 0 0
                0 1 0 0
                0 0 1 0
                ];

            local_vec = H * T * base_vec'; % was Nx4; now 3xN

            if nargout <= 1
                varargout{1} = local_vec'; % back to Nx3
            else
                for i = nargout:-1:1
                    varargout{i} = reshape(local_vec(i,:), orig_sz);
                end
            end
        end

        function reposition(obj, new_pos)
            %REPOSITION Set a new origin.
            arguments
                obj(1,1) ReferenceFrame3d
                new_pos(3,1) double {mustBeReal, mustBeFinite}
            end
            obj.T(1:3,4) = new_pos;
        end

        function reorient(obj, angles, opts)
            %REORIENT Explicitly assign rotation via a new euler sequence.
            arguments
                obj(1,1) ReferenceFrame3d
                angles(1,:) double {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.Sequence(1,:) char = 'zyx'
                opts.Units(1,1) string = "deg"
            end

            obj.T(1:3,1:3) = eye(3);
            obj.rotate_euler(angles,"Sequence",opts.Sequence,"Units",opts.Units);
        end

        function repose(obj, angles, position, opts)
            %REPOSE Explicitly assign a new orientation and position.
            arguments
                obj(1,1) ReferenceFrame3d
                angles(1,:) double
                position(1,3) double
                opts.Sequence(1,:) char = 'zyx'
                opts.Units(1,1) string = "deg"
            end

            obj.reorient(angles,"Sequence",opts.Sequence,"Units",opts.Units);
            obj.reposition(position);
        end

        function assign(obj, other)
            %ASSIGN Update the transform to match another ReferenceFrame3d object.
            arguments
                obj(1,1) ReferenceFrame3d
                other(1,1) ReferenceFrame3d
            end

            obj.T = other.as_transform();
        end

        function translate(obj, dxyz)
            %TRANSLATE Shift the origin by an incremental amount.
            arguments
                obj(1,1) ReferenceFrame3d
                dxyz(3,1) double {mustBeReal, mustBeFinite}
            end

            obj.T(1:3,4) = obj.T(1:3,4) + dxyz;
        end

        function rotate_euler(obj, angles, opts)
            %ROTATE_EULER Rotate by an euler sequence.
            arguments
                obj(1,1) ReferenceFrame3d
                angles(1,:) double {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.Sequence(1,:) char = 'zyx'
                opts.Units(1,1) string = "deg"
            end

            T_sequence = ReferenceFrame3d.euler_transform(angles, ...
                "Sequence", opts.Sequence, ...
                "Units", opts.Units);
            obj.T = obj.T * T_sequence;
        end

        function rotate_dcm(obj, dcm)
            %ROTATE_DCM Rotate with a 3x3 Direction Cosine Matrix (DCM).
            arguments
                obj(1,1) ReferenceFrame3d
                dcm(3,3) double
            end
            T = dcm; %#ok<*PROPLC>
            T(4,4) = 1; % 3x3 -> 4x4 with no translation
            obj.T = obj.T * T;
        end

        function new = compose(obj)
            %COMPOSE Multiply through a sequence of transforms T = T1*T2*T3...
            arguments (Repeating)
                obj(:,1) ReferenceFrame3d
            end
            obj = vertcat(obj{:});
            T_new = obj(end).T;
            for i = numel(obj)-1:-1:1
                T_new = obj(i).T * T_new;
            end
            new = ReferenceFrame3d(T_new);
        end

        function new = inv(obj)
            %INV Inverse of the transform.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            R_inv = obj.R'; % transpose = inverse for a DCM by definition
            t_inv = -R_inv * obj.origin;
            new = ReferenceFrame3d([R_inv, t_inv; 0 0 0 1]);
        end
    end

    methods
        function [p, dist] = intersect_plane(obj, observer, ray, opts)
            %INTERSECT_PLANE Fire a ray at a 2D slice of the reference frame.
            arguments
                obj(1,1) ReferenceFrame3d
                observer(1,3) double
                ray(1,3) double
                opts.Slice(1,2) char = 'xy'
                opts.Offset(1,1) double = 0
                opts.Tol(1,1) double = 1e-6
                opts.Debug(1,1) logical = false
            end

            % select the 2 basis vectors that define the plane
            switch opts.Slice
                case 'xy'
                    a = obj.x; b = obj.y;
                case 'xz'
                    a = obj.x; b = obj.z;
                case 'yz'
                    a = obj.y; b = obj.z;
                case 'yx'
                    a = obj.y; b = obj.x;
                case 'zx'
                    a = obj.z; b = obj.x;
                case 'zy'
                    a = obj.z; b = obj.y;
                otherwise
                    validatestring(opts.Slice,{'xy','xz','yz','yx','zx','zy'});
            end

            normal = cross(a, b);
            normal = normal ./ norm(normal);
            point = obj.origin + (opts.Offset * normal);

            denominator = dot(normal, ray);

            if abs(denominator) < opts.Tol
                p = [nan nan nan];
                dist = nan;
                return
            end

            numerator = dot(normal(:), point(:) - observer(:));
            dist = numerator / denominator;
            p = observer + (dist * ray);

            if opts.Debug
                % create the figure & basis vectors
                ax = obj.show();

                % try to size the plane in a way that makes sense for the problem
                sz = unique(abs(p(p ~= 0))*4);
                if isempty(sz), sz = [4 4]; end
                if numel(sz) > 2, sz = maxk(sz, 2)*4; end

                % draw the plane & intersecting line
                obj.draw_plane('Slice', opts.Slice, ...
                    'Offset', opts.Offset, ...
                    'Size', sz);
                plot3([p(1) observer(1)], [p(2) observer(2)], [p(3) observer(3)], ...
                    'r-', ...
                    'Marker', 'o', ...
                    'MarkerFaceColor','k', ...
                    'Parent', ax);
            end
        end
    end

    %% Numeric Representations
    methods
        function T = as_transform(obj)
            %AS_TRANSFORM Express as a 4x4 homogeneous transform.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            T = obj.T;
        end

        function R = as_dcm(obj)
            %AS_DCM Express as a 3x3 pure rotation matrix.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            R = obj.R;
        end

        function [yaw, pitch, roll] = as_euler(obj, angle_unit)
            %AS_EULER Express as euler angles (zyx sequence).
            arguments
                obj(1,1) ReferenceFrame3d
                angle_unit(1,1) string = "deg"
            end

            assert(contains(angle_unit, ["deg","degrees","rad","radians"]), ...
                'Unknown angle unit (expected "deg", "degrees", "rad", "radians"; got "%s")', ...
                angle_unit);

            % use R_base_to_local (transpose of the frame's R) for calculations
            % consistent with many standard derivations.
            R = obj.R';
            
            % define a tolerance slightly less than 1 for gimbal lock check
            gimbal_tol = 1 - 1e-9;

            % check for gimbal lock (pitch = ±90°) using R'(1,3)
            if abs(R(1,3)) > gimbal_tol
                % in gimbal lock, yaw and roll are not uniquely determined.
                % set yaw to 0 by convention and calculate roll accordingly.
                yaw = 0;

                if R(1,3) < 0 % corresponds to pitch = +90°
                    pitch = pi/2;
                    roll = atan2(R(2,1), R(2,2));
                else % corresponds to pitch = -90°
                    pitch = -pi/2;
                    roll = -atan2(R(2,1), R(2,2)); % keep based on common gimbal lock solutions
                end
            else
                % normal case - no gimbal lock
                pitch = asin(-R(1,3)); % pitch = asin(-R'(1,3))
                yaw = atan2(R(1,2), R(1,1)); % yaw = atan2(R'(1,2), R'(1,1))
                roll = atan2(R(2,3), R(3,3)); % roll = atan2(R'(2,3), R'(3,3))
            end

            % convert to requested units
            if contains(angle_unit, "deg")
                yaw = yaw * 180/pi;
                pitch = pitch * 180/pi;
                roll = roll * 180/pi;
            end
        end
    end

    %% MATLAB Toolbox Interoperability
    %
    %   These methods are subject to the availability of toolboxes on your system.

    methods
        function tform = se3(obj)
            %SE3 Convert to an SE3 object.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            tform = se3(obj.T);
        end

        function rot = so3(obj)
            %SO3 Convert to an SO3 object.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            rot = so3(obj.R);
        end

        function q = quaternion(obj)
            %QUATERNION Convert to a quaternion object.
            arguments
                obj(1,1) ReferenceFrame3d
            end
            q = quaternion(obj.R,"rotmat","point");
        end
    end

    %% Overloads
    methods
        function new = mtimes(obj, other)
            %MTIMES Matrix multiplication.
            arguments
                obj(1,1) ReferenceFrame3d
                other(1,1) ReferenceFrame3d
            end
            new = ReferenceFrame3d(obj.T * other.T);
        end
    end

    %% Graphics
    methods (Sealed)
        function ax = show(obj)
            %SHOW Plot everything in a new, dedicated figure.
            arguments
                obj(:,1) ReferenceFrame3d
            end

            hfig = figure;
            ax = axes('parent', hfig);
            grid(ax, 'on');
            box(ax, 'on');

            % configure line lengths
            obj.plot(...
                'Parent',ax, ...
                'TextLabels', true, ...
                'LineLength', max(1,local_find_max_origin_dist(obj)/4));

            hold(ax,'on');
            axis(ax, 'equal');
            view(ax,45,30);
            rotate3d(ax,'on');

            function maxr = local_find_max_origin_dist(frames)
                maxr = nan;
                for i = 1:numel(frames)
                    composed = frames(1:i).compose();
                    maxr = max(maxr, vecnorm(composed.origin));
                end
            end
        end

        function plot(objs, opts)
            %PLOT Plot the frame's basis vectors in 3d.
            arguments
                objs(:,1) ReferenceFrame3d
                opts.Parent = []
                opts.Colors(1,3) char = 'rgb' % for basis xyz
                opts.LineWidth(1,3) double = 1
                opts.LineStyle(1,:) char = '-'
                opts.LineLength(1,3) double = 1
                opts.Arrowheads(1,3) matlab.lang.OnOffSwitchState = true
                opts.TextLabels(1,3) matlab.lang.OnOffSwitchState = false
                opts.ConnectFrames(1,1) matlab.lang.OnOffSwitchState = true
                opts.Detach(1,1) logical = false % create no ties to original objects
                opts.Delete(1,1) logical = false % delete and return early
            end

            if opts.Detach
                for i = 1:numel(objs)
                    objs(i) = ReferenceFrame3d(objs(i)); % overwrite with a copy
                end
            end

            % create the transforms (if they don't already exist)
            objs.hgtransform(opts.Parent);
            sz = opts.LineLength;

            for i = 1:numel(objs)
                % plot each coordinate system relative to its base frame
                parent = objs(i).h_transform;
                
                ax = ancestor(parent, 'axes');
                assert(~isempty(ax), ...
                    'The ancestor axis to the hgtransform is empty.');
    
                % we may wish to plot other things to the reference frame transform,
                % so let's organize the basis vector data under its own group
                delete(objs(i).h_plot_group);
                if opts.Delete, continue; end
                objs(i).h_plot_group = hggroup('Parent', parent, 'Tag', 'RF3D_BASIS_GROUP');

                if opts.ConnectFrames && i < numel(objs)
                    % draw an arrow connecting the current frame to the next
                    line([0 objs(i+1).origin(1)], [0 objs(i+1).origin(2)], [0 objs(i+1).origin(3)], ...
                        'Parent', objs(i).h_plot_group, ...
                        'Color', 'k', ...
                        'LineStyle', '--', ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_FRAME_CONNECTOR');
                end
    
                if all(opts.LineWidth == opts.LineWidth(1)) && all(opts.Colors == opts.Colors(1))
                    % interleave NaNs to plot all basis vectors as a single object
                    xdata = [0 sz(1) NaN; 0 0 NaN; 0 0 NaN]';
                    ydata = [0 0 NaN; 0 sz(2) NaN; 0 0 NaN]';
                    zdata = [0 0 NaN; 0 0 NaN; 0 sz(3) NaN]';
                    line(xdata(:), ydata(:), zdata(:), ...
                        'Parent', objs(i).h_plot_group, ...
                        'Color', opts.Colors(1), ...
                        'LineWidth', opts.LineWidth(1), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                else
                    line([0 sz(1)], [0 0], [0 0], ...
                        'Parent', objs(i).h_plot_group, ...
                        'Color', opts.Colors(1), ...
                        'LineWidth', opts.LineWidth(1), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                    line([0 0], [0 sz(2)], [0 0], ...
                        'Parent', objs(i).h_plot_group, ...
                        'Color', opts.Colors(2), ...
                        'LineWidth', opts.LineWidth(2), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                    line([0 0], [0 0], [0 sz(3)], ...
                        'Parent', objs(i).h_plot_group, ...
                        'Color', opts.Colors(3), ...
                        'LineWidth', opts.LineWidth(3), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                end
                basis_letter = 'xyz';
                for j = 1:3
                    if opts.Arrowheads(j)
                        plot_arrowhead(objs(i).h_plot_group, sz(j), opts.Colors(j), j);
                    end
                    if opts.TextLabels(j)
                        if ~ismissing(objs(i).name)
                            name = ['_{' objs(i).name{1} '}'];
                        else
                            name = '';
                        end
                        text(sz(1)*(j==1), sz(2)*(j==2), sz(3)*(j==3), ...
                            string([' ' basis_letter(j) name]), ...
                            'Parent', objs(i).h_plot_group, ...
                            'Color', opts.Colors(j), ...
                            'HitTest','off', ...
                            'PickableParts','none');
                    end
                end
                line(0, 0, 0, ...
                    'Parent', objs(i).h_plot_group, ...
                    'Color', 'k', ...
                    'Marker', '.', ...
                    'MarkerSize', 12, ...
                    'Hittest','off','PickableParts','none',...
                    'Clipping', 'off');
            end

            function plot_arrowhead(parent, base_length, color, basis)
                %PLOT_ARROWHEAD Local function to plot a colored cone.
                m = 20; n = 2; headsz = 0.1;
                theta = linspace(0, 2*pi, m);
                R = linspace(0, 0.2*base_length, n);
                [T, R] = meshgrid(theta, R);
    
                switch basis
                    case 1 % x
                        x = base_length - R;
                        y = R.*headsz.*sin(T);
                        z = R.*headsz.*cos(T);
                    case 2 % y
                        y = base_length - R;
                        z = R.*headsz.*sin(T);
                        x = R.*headsz.*cos(T);
                    case 3 % z
                        z = base_length - R;
                        x = R.*headsz.*sin(T);
                        y = R.*headsz.*cos(T);
                end

                surface(x, y, z, ...
                    'Parent', parent, ...
                    'Clipping', 'off', ...
                    'EdgeColor', 'none', ...
                    'FaceColor', color, ...
                    'HitTest', 'off', 'PickableParts', 'none');
            end
        end

        function h = draw_box(obj, opts)
            %DRAW_BOX Draw a rectangular prism aligned to the basis vectors of the frame.
            arguments
                obj(1,1) ReferenceFrame3d
                opts.Dimensions(1,3) double {mustBeFinite} = [1 1 1] % xyz (length)
                opts.Center(1,3) double {mustBeFinite} = [0 0 0] % xyz (center)
                opts.FaceColor = 0.6*[1 1 1]
                opts.FaceAlpha = 0.2
                opts.EdgeColor = 0.2*[1 1 1]
                opts.EdgeAlpha = 0.2
                opts.LineStyle(1,:) char = '-'
                opts.Clipping(1,1) matlab.lang.OnOffSwitchState = 'off'
            end

            validateattributes(opts.Dimensions, {'double'},{'>',0,'real'});

            % half lengths
            hx = opts.Dimensions(1) / 2; 
            hy = opts.Dimensions(2) / 2; 
            hz = opts.Dimensions(3) / 2;
        
            % define the 2x2 coordinate matrices for each face in order of
            % bottom, top, front, back, left, right
            Xb=[-hx +hx; -hx +hx]; Yb=[-hy -hy; +hy +hy]; Zb=[-hz -hz; -hz -hz];
            Xt=[-hx +hx; -hx +hx]; Yt=[-hy -hy; +hy +hy]; Zt=[+hz +hz; +hz +hz];
            Xf=[-hx +hx; -hx +hx]; Yf=[-hy -hy; -hy -hy]; Zf=[-hz -hz; +hz +hz];
            Xk=[-hx +hx; -hx +hx]; Yk=[+hy +hy; +hy +hy]; Zk=[-hz -hz; +hz +hz];
            Xl=[-hx -hx; -hx -hx]; Yl=[-hy +hy; -hy +hy]; Zl=[-hz -hz; +hz +hz];
            Xr=[+hx +hx; +hx +hx]; Yr=[-hy +hy; -hy +hy]; Zr=[-hz -hz; +hz +hz];
        
            % combine into larger matrices using NaNs to separate
            % (arrange faces conceptually in a 2x3 grid within the matrices)
            nan_col = NaN(2,1); % NaN column separator
            row1X = [Xb, nan_col, Xt, nan_col, Xf]; % 3 faces side-by-side
            row1Y = [Yb, nan_col, Yt, nan_col, Yf];
            row1Z = [Zb, nan_col, Zt, nan_col, Zf];
        
            nan_row = NaN(1, size(row1X, 2)); % NaN row separator
        
            row2X = [Xk, nan_col, Xl, nan_col, Xr]; % next 3 faces side-by-side
            row2Y = [Yk, nan_col, Yl, nan_col, Yr];
            row2Z = [Zk, nan_col, Zl, nan_col, Zr];
        
            % final matrices for the single surface object
            X = opts.Center(1) + [row1X; nan_row; row2X];
            Y = opts.Center(2) + [row1Y; nan_row; row2Y];
            Z = opts.Center(3) + [row1Z; nan_row; row2Z];
        
            h = surface(X, Y, Z, ...
                'FaceColor', opts.FaceColor, ...
                'FaceAlpha', opts.FaceAlpha, ...
                'EdgeColor', opts.EdgeColor, ...
                'EdgeAlpha', opts.EdgeAlpha, ...
                'LineStyle', opts.LineStyle, ...
                'Clipping', opts.Clipping, ...
                'HitTest','off', ...
                'PickableParts','none', ...
                'Tag', sprintf('BOX (%.3f x %.3f x %.3f)', opts.Dimensions), ...
                'Parent', hgtransform(obj));
        end

        function h = draw_plane(obj, opts)
            %DRAW_PLANE Draw a plane in the axis.
            arguments
                obj(1,1) ReferenceFrame3d
                opts.Slice(1,2) char = 'xy'
                opts.Size(1,2) double {mustBeReal, mustBeFinite} = [4 4] % [x y]
                opts.GridLineSpacing(1,2) double = [nan nan] % [x y]
                opts.Datum(1,2) double = [nan nan] % [x y]
                opts.Offset(1,1) double = 0
                opts.FaceColor = 0.6*[1 1 1]
                opts.FaceAlpha = 0.2
                opts.EdgeColor = 0.2*[1 1 1]
                opts.EdgeAlpha = 0.2
                opts.LineStyle(1,:) char = '-'
                opts.Clipping(1,1) matlab.lang.OnOffSwitchState = 'off'
            end

            % override default options
            inan = isnan(opts.GridLineSpacing);
            opts.GridLineSpacing(inan) = opts.Size(inan); % no grid lines by default
            opts.GridLineSpacing = min([opts.GridLineSpacing; opts.Size], [], 1);
            inan = isnan(opts.Datum);
            opts.Datum(inan) = -opts.Size(inan) ./ 2; % center the plane by default

            grid_x = 0 : opts.GridLineSpacing(1) : opts.Size(1);
            grid_y = 0 : opts.GridLineSpacing(2) : opts.Size(2);
            if grid_x(end) ~= opts.Size(1)
                grid_x(end+1) = opts.Size(1);
            end
            if grid_y(end) ~= opts.Size(2)
                grid_y(end+1) = opts.Size(2);
            end
            grid_x = grid_x + opts.Datum(1);
            grid_y = grid_y + opts.Datum(2);

            [xdata, ydata] = meshgrid(grid_x, grid_y);
            zdata = repmat(opts.Offset, size(xdata));

            % our plane is defined in the x-y plane, but user may have requested a
            % different slice.  so we'll define a new frame co-located with the
            % current frame that moves the basis vectors around as required
            switch opts.Slice
                case 'xy'
                    a = [1 0 0]'; b = [0 1 0]';
                case 'xz'
                    a = [1 0 0]'; b = [0 0 1]';
                case 'yz'
                    a = [0 1 0]'; b = [0 0 1]';
                case 'yx'
                    a = [0 1 0]'; b = [1 0 0]';
                case 'zx'
                    a = [0 0 1]'; b = [1 0 0]';
                case 'zy'
                    a = [0 0 1]'; b = [0 1 0]';
                otherwise
                    validatestring(opts.Slice,{'xy','xz','yz','yx','zx','zy'});
            end
            
            plane = ReferenceFrame3d([a b cross(a, b)], [0 0 0]);
            [xdata, ydata, zdata] = plane.local2base(xdata, ydata, zdata);

            h = surface(...
                'XData', xdata, ...
                'YData', ydata, ...
                'ZData', zdata, ...
                'FaceColor', opts.FaceColor, ...
                'FaceAlpha', opts.FaceAlpha, ...
                'EdgeColor', opts.EdgeColor, ...
                'EdgeAlpha', opts.EdgeAlpha, ...
                'LineStyle', opts.LineStyle, ...
                'Clipping', opts.Clipping, ...
                'HitTest','off', ...
                'PickableParts','none', ...
                'Tag', sprintf('PLANE (%s)', upper(opts.Slice)), ...
                'Parent', hgtransform(obj));
        end

        function tform = hgtransform(objs, parent)
            %HGTRANSFORM Get/create the graphics transforms.
            arguments
                objs(:,1) ReferenceFrame3d
                parent = []
            end

            if isa(parent, 'ReferenceFrame3d')
                assert(~isempty(parent.h_transform) && isvalid(parent.h_transform), ...
                    'Invalid parent (no transform has been created)');
                parent = parent.h_transform;
            end

            % consider the base transform invalid if the parent has changed
            if ~isempty(parent) && isvalid(parent) ...
                    && ~isempty(objs(1).h_transform) ...
                    && isvalid(objs(1).h_transform) ...
                    && ~isequal(objs(1).h_transform.Parent, parent)
                % delete ALL graphics objects (start over)
                objs.clear();
            end

            % create new axes only if we don't have a parent to target
            % AND there's no valid transform already
            if (isempty(parent) || ~isvalid(parent)) ...
                    && (isempty(objs(1).h_transform) || ~isvalid(objs(1).h_transform))
                parent = gca();
            end

            % create a new transform for each object (only if necessary)
            for i = 1:numel(objs)
                % skip if a transform already exists and is parented correctly
                % (parented to the previous transform in the sequence)
                if ~isempty(objs(i).h_transform) && isvalid(objs(i).h_transform)
                    continue
                end
                if i > 1 ...
                        && ~isempty(objs(i).h_transform) && isvalid(objs(i).h_transform) ...
                        && isequal(objs(i).h_transform.Parent, objs(i-1).h_transform)
                    continue
                end
                delete(objs(i).h_transform);
                objs(i).h_transform = hgtransform('Parent', parent, 'Matrix', objs(i).T);
                parent = objs(i).h_transform;
            end

            tform = objs(end).h_transform;
        end

        function update_hgtransform(objs)
            %UPDATE_HGTRANSFORM Update the graphics transform to match the object state.
            arguments
                objs(:,1) ReferenceFrame3d
            end
            for i = 1:numel(objs)
                if isempty(objs(i).h_transform) || ~isvalid(objs(i).h_transform)
                    continue
                end
                objs(i).h_transform.Matrix = objs(i).T;
            end
        end

        function clear(objs)
            %CLEAR Delete all graphics objects.
            arguments
                objs(:,1) ReferenceFrame3d
            end
            for i = 1:numel(objs)
                delete(objs(i).h_transform);
                objs(i).h_transform = matlab.graphics.primitive.Transform.empty;
                objs(i).h_plot_group = matlab.graphics.primitive.Group.empty;
            end
        end
    end

    %% Static helpers
    methods (Static)
        function T_sequence = euler_transform(angles, opts)
            %EULER_TRANSFORM Get the transformation for an euler sequence.
            arguments
                angles(1,:) double {mustBeNumeric, mustBeReal, mustBeFinite}
                opts.Sequence(1,:) char = 'zyx' % yaw-pitch-roll
                opts.Units(1,1) string = "deg"
            end

            sequence = lower(opts.Sequence);
            assert(numel(angles) == numel(opts.Sequence), ...
                'Expected the number of angles (%d) to match number of rotations (%d).',...
                numel(angles), numel(opts.Sequence));
            assert(all(ismember(opts.Sequence, 'xyz')), ...
                'Rotations may only be specified as only ''x'', ''y'', or ''z''');
            assert(contains(opts.Units, ["deg","degrees","rad","radians"]), ...
                'Unknown angle unit (expected "deg", "degrees", "rad", "radians"; got "%s")', ...
                opts.Units);
            
            if any(contains(opts.Units, ["deg", "degrees"]))
                angles_rad = angles * pi/180;
            else
                angles_rad = angles;
            end

            T_sequence = eye(4);

            for i = 1:numel(opts.Sequence)
                axis_char = sequence(i);
                angle_val = angles_rad(i);
                
                c = local_fixup(cos(angle_val));
                s = local_fixup(sin(angle_val));

                T_axis = eye(4);
                switch axis_char
                    case 'x', T_axis(2:3, 2:3) = [c, -s; s, c];
                    case 'y', T_axis([1,3], [1,3]) = [c, s; -s, c];
                    case 'z', T_axis(1:2, 1:2) = [c, -s; s, c];
                end
                
                T_sequence = T_sequence * T_axis;
            end

            function x_fixed = local_fixup(x)
                % snap to 0, 1, -1 (if very close) to improve numerical stability
                x_fixed = x;
                snap_threshold = 2*eps;
                if abs(x) < snap_threshold, x_fixed = 0.0;
                elseif abs(x - 1.0) < snap_threshold, x_fixed = 1.0;
                elseif abs(x + 1.0) < snap_threshold, x_fixed = -1.0;
                end
            end
        end
    end

    %% Property accessors

    % set
    methods
        function set.T(obj, T_new)
            ReferenceFrame3d.validate_transform(T_new);
            obj.T = T_new;
            obj.update_hgtransform();
        end
    end

    % get
    methods
        function v = get.R(obj)
            v = obj.T(1:3,1:3);
        end

        function v = get.x(obj)
            v = obj.T(1:3,1);
        end

        function v = get.y(obj)
            v = obj.T(1:3,2);
        end

        function v = get.z(obj)
            v = obj.T(1:3,3);
        end

        function v = get.origin(obj)
            v = obj.T(1:3,4);
        end
    end

    %% matlab.mixin.Copyable
    methods (Access = protected)
        function copied = copyElement(obj)
            copied = copyElement@matlab.mixin.Copyable(obj);

            % only copy graphics objects if something has been plotted
            if isempty(obj.h_transform) || ~isvalid(obj.h_transform)
                return
            end
            parent = obj.h_transform.Parent;
            copied.h_transform = copyobj(obj.h_transform, parent);
            copied.h_plot_group = copyobj(obj.h_plot_group, parent);
        end
    end

    %% matlab.mixin.CustomDisplay
    methods (Sealed, Access = protected)
        function footer = getFooter(obj)
            deg = char(176);

            base_str = 'base';
            local_str = 'local';

            if isscalar(obj)
                footer = '';
                frame = obj;
            else
                % don't display a custom footer for massive or multi-dim arrays
                if numel(obj) > 100
                    footer = '   This array represents a transformation sequence too large to display.'; 
                    return
                end

                if ismissing(obj(1).name) || ismissing(obj(end).name)
                    footer = '   This array represents a transformation sequence.';
                else
                    base_str = sprintf('<strong>%s</strong>', obj(1).name);
                    local_str = sprintf('<strong>%s</strong>', obj(end).name);
                    footer = sprintf(...
                        '   This array represents a transformation sequence from %s to %s.', ...
                        base_str, local_str);
                end

                footer = sprintf('%s  When composed:\n\n', footer);

                frame = compose(obj);
            end

            [y,p,r] = frame.as_euler("deg");

            footer = sprintf(...
                ['%s' ...
                '   origin (of %s, expressed in %s frame):\n' ...
                '\n' ...
                '       %14.9f\n' ...
                '       %14.9f\n' ...
                '       %14.9f\n' ...
                '\n' ...
                '   orientation (zyx euler sequence %s -> %s):\n' ...
                '\n' ...
                '       yaw   = %14.9f%s\n' ...
                '       pitch = %14.9f%s\n' ...
                '       roll  = %14.9f%s\n'], ...
                footer, ...
                local_str, base_str, ...
                frame.origin, ...
                local_str, base_str, ...
                round(y+eps,9), deg, round(p+eps,9), deg, round(r+eps,9), deg);
        end
    end

end
