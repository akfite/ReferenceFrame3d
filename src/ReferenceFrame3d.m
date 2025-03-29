classdef ReferenceFrame3d < matlab.mixin.Copyable ...
        & matlab.mixin.CustomDisplay
    
    properties
        T(4,4) double = eye(4) % homogeneous transform (rotation & translation)
    end

    properties (Dependent)
        R(3,3) double % rotation submatrix
        x(3,1) double % x basis
        y(3,1) double % y basis
        z(3,1) double % z basis
        t(3,1) double % translation (origin of reference frame)
    end

    % graphics
    properties (Access = protected, Transient)
        h_transform matlab.graphics.primitive.Transform
        h_plot_group matlab.graphics.primitive.Group
    end

    %% Construction
    methods
        function this = ReferenceFrame3d(matrix, origin)
            %REFERENCEFRAME3D Constructor.
            arguments
                matrix double = eye(3)
                origin(1,3) double = [0 0 0]
            end
            this.setup(matrix, origin);
        end

        function this = setup(this, matrix, origin)
            %SETUP Configure an existing object with a 3x3 DCM & origin vector.
            arguments
                this(1,1) ReferenceFrame3d
                matrix double
                origin(1,3) double = [0 0 0]
            end

            if isequal(size(matrix,[1 2]), [4 4])
                T = matrix;
            else
                matrix(4,4) = 1;
                T = matrix * makehgtform('translate', origin);
            end

            this.T = T;
        end
    end

    methods (Static)
        function this = from_point_normal(point, normal)
            %FROM_POINT_NORMAL Creates a new frame to represent a plane.
            arguments
                point(1,3) double {mustBeReal, mustBeFinite}
                normal(1,3) double {mustBeReal, mustBeFinite}
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
            this = ReferenceFrame3d(T);
        end

        function this = from_coplanar_vectors(x_basis, y_basis, origin)
            %TODO
        end

        function validate_transform(T)
            %VALIDATE Check that the transform makes sense as configured.
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
        end
    end

    %% Math & utility
    methods (Sealed)
        function varargout = local2base(this, varargin)
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

            if isscalar(this)
                T = this.T;
            else
                T = as_matrix(compose(this)); % combine rotations
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

        function varargout = base2local(this, varargin)
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

            if isscalar(this)
                T = as_matrix(inv(this));
            else
                T = as_matrix(inv(compose(this))); % combine rotations
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

        function this = translate(this, dxyz)
            %TRANSLATE Shift the origin by an incremental amount.
            arguments
                this(1,1) ReferenceFrame3d
                dxyz(3,1) double {mustBeReal, mustBeFinite}
            end
            this.T(1:3,4) = this.T(1:3,4) + dxyz;
        end

        function this = reposition(this, new_pos)
            arguments
                this(1,1) ReferenceFrame3d
                new_pos(3,1) double {mustBeReal, mustBeFinite}
            end
            this.T(1:3,4) = new_pos;
        end
        
        function this = rotate(this, dcm)
            arguments
                this(1,1) ReferenceFrame3d
                dcm(3,3) double
            end
            T = dcm; %#ok<*PROPLC>
            T(4,4) = 1; % 3x3 -> 4x4 with no translation
            this.T = this.T * T;
        end

        function this = rotate_euler(this, roll, pitch, yaw)
            cr = cos(roll);
            sr = sin(roll);
            cp = cos(pitch);
            sp = sin(pitch);
            cy = cos(yaw);
            sy = sin(yaw);
    
            T = [...
                          cy*cp,              sy*cp,      -sp,       0;
                -sy*cr+cy*sp*sr,     cy*cr+sy*sp*sr,    cp*sr,       0;
                 sy*sr+cy*sp*cr,    -cy*sr+sy*sp*cr,    cp*cr,       0
                              0,                  0,        0,       1];

            this.T = this.T * T;
        end

        function this = rotate_eulerd(this, roll, pitch, yaw)
            this.rotate_euler(roll * pi/180, pitch * pi/180, yaw * pi/180);
        end

        function new = compose(this)
            arguments (Repeating)
                this(:,1) ReferenceFrame3d
            end
            this = vertcat(this{:});
            T_new = this(end).T;
            for i = numel(this)-1:-1:1
                T_new = this(i).T * T_new;
            end
            new = ReferenceFrame3d(T_new);
        end

        function this = inv(this)
            R_inv = this.R'; % transpose = inverse for a DCM by definition
            t_inv = -R_inv * this.t;
            this.T = [R_inv, t_inv; 0 0 0 1];
        end

        function T = as_matrix(this)
            arguments
                this(1,1) ReferenceFrame3d
            end
            T = this.T;
        end

        function R = as_dcm(this)
            arguments
                this(1,1) ReferenceFrame3d
            end
            R = this.R;
        end

        function q = as_quaternion(this)
            try
            catch me
                if strcmp(me.identifier, 'MATLAB:ErrorRecovery:UnlicensedFunction')
                end
            end
        end

        function [roll, pitch, yaw] = as_euler(this)
            dcm = this.R;
            roll = atan2(dcm(2,3,:), dcm(3,3,:));
            pitch = asin(-dcm(1,3,:));
            yaw = atan2(dcm(1,2,:), dcm(1,1,:));
        end

        function [roll, pitch, yaw] = as_eulerd(this)
            [roll, pitch, yaw] = as_euler(this);
            roll = roll * 180/pi;
            pitch = pitch * 180/pi;
            yaw = yaw * 180/pi;
        end
    end

    methods
        function [p, dist] = intersect_plane(this, observer, ray, opts)
            arguments
                this(1,1) ReferenceFrame3d
                observer(1,3) double
                ray(1,3) double
                opts.Slice(1,2) char = 'xy'
                opts.Offset(1,1) double = 0
                opts.Tol(1,1) double = 1e-6
                opts.Debug(1,1) logical = true
            end

            % select the 2 basis vectors that define the plane
            switch opts.Slice
                case 'xy'
                    a = this.x; b = this.y;
                case 'xz'
                    a = this.x; b = this.z;
                case 'yz'
                    a = this.y; b = this.z;
                case 'yx'
                    a = this.y; b = this.x;
                case 'zx'
                    a = this.z; b = this.x;
                case 'zy'
                    a = this.z; b = this.y;
                otherwise
                    validatestring(opts.Slice,{'xy','xz','yz','yx','zx','zy'});
            end

            normal = cross(a, b);
            normal = normal ./ norm(normal);
            point = this.t + (opts.Offset * normal);

            denominator = dot(normal, ray);

            if abs(denominator) < opts.Tol
                p = [nan nan nan];
                return
            end

            numerator = dot(normal, point - observer);
            dist = numerator / denominator;
            p = observer + (dist * ray);

            if opts.Debug
                % TODO
            end
        end
    end

    %% Overloads
    methods
        function new = mtimes(this, other)
            %MTIMES Matrix multiplication.
            new = ReferenceFrame3d(this.T * other.T);
        end

        function this = ctranspose(this)
            transpose(this); % complex types are not supported (' == .')
        end

        function this = transpose(this)
            this.T(1:3,1:3) = this.T(1:3,1:3).';
        end
    end

    %% Graphics
    methods (Sealed)
        function show(this)
            %SHOW Draw everything in a new, dedicated figure

            hfig = figure;
            ax = axes('parent', hfig);
            grid(ax, 'on');
            box(ax, 'on');

            this.plot('parent',ax);
        end

        function plot(objs, opts)
            %PLOT Plot as a 3-d object.
            arguments
                objs(:,1) ReferenceFrame3d
                opts.Parent = []
                opts.Colors(1,3) char = 'rgb' % for basis xyz
                opts.LineWidth(1,3) double = 1
                opts.LineStyle(1,:) char = '-'
                opts.LineLength(1,3) double = 1
                opts.EnableArrowheads(1,3) matlab.lang.OnOffSwitchState = 1
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
                objs(i).h_plot_group = hggroup('Parent', parent, 'Tag', 'RF3D_BASIS_GROUP');
    
                if all(opts.LineWidth == opts.LineWidth(1)) && all(opts.Colors == opts.Colors(1))
                    % interleave NaNs to plot all basis vectors as a single object
                    xdata = [0 sz(1) NaN; 0 0 NaN; 0 0 NaN]';
                    ydata = [0 0 NaN; 0 sz(2) NaN; 0 0 NaN]';
                    zdata = [0 0 NaN; 0 0 NaN; 0 sz(3) NaN]';
                    line(objs(i).h_plot_group, ...
                        xdata(:), ydata(:), zdata(:), ...
                        'Color', opts.Colors(1), ...
                        'LineWidth', opts.LineWidth(1), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                else
                    line(objs(i).h_plot_group, ...
                        [0 sz(1)], [0 0], [0 0], ...
                        'Color', opts.Colors(1), ...
                        'LineWidth', opts.LineWidth(1), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                    line(objs(i).h_plot_group, ....
                        [0 0], [0 sz(2)], [0 0], ...
                        'Color', opts.Colors(2), ...
                        'LineWidth', opts.LineWidth(2), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                    line(objs(i).h_plot_group, ...
                        [0 0], [0 0], [0 sz(3)], ...
                        'Color', opts.Colors(3), ...
                        'LineWidth', opts.LineWidth(3), ...
                        'LineStyle', opts.LineStyle, ...
                        'Clipping', 'off', ...
                        'Tag', 'RF3D_BASIS_VECTORS');
                end
                for j = 1:3
                    if opts.EnableArrowheads(j)
                        plot_arrowhead(objs(i).h_plot_group, sz(j), opts.Colors(j), j);
                    end
                end
                line(objs(i).h_plot_group, ...
                    'XData', 0, 'YData', 0, 'ZData', 0, ...
                    'Color', 'k', ...
                    'Marker', '.', ...
                    'MarkerSize', 12, ...
                    'Hittest','off','PickableParts','none',...
                    'Clipping', 'off');
            end

            function plot_arrowhead(parent, base_length, color, basis)
                m = 20; n = 2;
                theta = linspace(0, 2*pi, m);
                R = linspace(0, 0.2*base_length, n);
                [T, R] = meshgrid(theta, R);
    
                switch basis
                    case 1
                        x = base_length - R;
                        y = R.*.2.*sin(T);
                        z = R.*.2.*cos(T);
                    case 2
                        y = base_length - R;
                        z = R.*.2.*sin(T);
                        x = R.*.2.*cos(T);
                    case 3
                        z = base_length - R;
                        x = R.*.2.*sin(T);
                        y = R.*.2.*cos(T);
                end

                surface(x, y, z, ...
                    'Parent', parent, ...
                    'Clipping', 'off', ...
                    'EdgeColor', 'none', ...
                    'FaceColor', color, ...
                    'HitTest', 'off', 'PickableParts', 'none');
            end
        end

        function h = draw_plane(obj, opts)
            %DRAW_PLANE Draw a plane in the axis.
            arguments
                obj(1,1) ReferenceFrame3d
                opts.Slice(1,2) char = 'xy'
                opts.Size(1,2) double {mustBeReal, mustBeFinite} = [4 4] % [x y]
                opts.GridLineSpacing(1,2) double = [nan nan] % [x y]
                opts.Datum(1,2) double = [nan nan] % [x y]
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
            zdata = zeros(size(xdata));

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
                'HitTest','off',...
                'PickableParts','none',...
                'Tag', sprintf('%s_PLANE', upper(opts.Slice)), ...
                'Parent', obj.h_transform);
        end

        function tform = hgtransform(objs, parent)
            %HGTRANSFORM Get the graphics transform paired to this object.

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
                        && ~isempty(objs(i).h_transform) ...
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
            %UPDATE_TRANSFORM Update the graphics transform to match the object state.
            for i = 1:numel(objs)
                if isempty(objs(i).h_transform) || ~isvalid(objs(i).h_transform)
                    continue
                end
                objs(i).h_transform.Matrix = objs(i).T;
            end
        end

        function clear(this)
            %CLEAR Delete all graphics objects.
            for i = 1:numel(this)
                delete(this(i).h_transform);
            end
        end
    end

    %% Property accessors

    % set
    methods
        function set.T(this, T_new)
            ReferenceFrame3d.validate_transform(T_new);
            this.T = T_new;
            this.update_hgtransform();
        end
    end

    % get
    methods
        function v = get.R(this)
            v = this.T(1:3,1:3);
        end

        function v = get.x(this)
            v = this.T(1:3,1);
        end

        function v = get.y(this)
            v = this.T(1:3,2);
        end

        function v = get.z(this)
            v = this.T(1:3,3);
        end

        function v = get.t(this)
            v = this.T(1:3,4);
        end
    end

    %% matlab.mixin.Copyable
    methods (Access = protected)
        function copied = copyElement(this)
            copied = copyElement@matlab.mixin.Copyable(this);

            % only copy graphics objects if something has been plotted
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                return
            end
            parent = this.h_transform.Parent;
            copied.h_transform = copyobj(this.h_transform, parent);
            copied.h_plot_group = copyobj(this.h_plot_group, parent);
        end
    end

    %% matlab.mixin.CustomDisplay
    methods (Sealed, Access = protected)
        function header = getHeader(obj)
            header = getHeader@matlab.mixin.CustomDisplay(obj);
        end

        function groups = getPropertyGroups(obj)
            groups = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
        end

        function footer = getFooter(this)
            deg = char(176);

            if isscalar(this)
                footer = '';
                frame = this;
            else
                footer = sprintf( ...
                    ['   This array represents a transformation sequence.  ' ...
                    'When composed:\n\n']);
                frame = compose(this);
            end

            [r,p,y] = frame.as_eulerd();

            footer = sprintf(...
                ['%s' ...
                '   origin (in base frame):\n' ...
                '\n' ...
                '       %14.9f\n' ...
                '       %14.9f\n' ...
                '       %14.9f\n' ...
                '\n' ...
                '   orientation (zyx euler sequence base->local):\n' ...
                '\n' ...
                '       roll  = %14.9f%s\n' ...
                '       pitch = %14.9f%s\n' ...
                '       yaw   = %14.9f%s\n'], ...
                footer, ...
                frame.t, round(r+eps,9), deg, round(p+eps,9), deg, round(y+eps,9), deg);
        end

        function displayNonScalarObject(obj)
            displayNonScalarObject@matlab.mixin.CustomDisplay(obj);
        end

        function displayScalarObject(obj)
            displayScalarObject@matlab.mixin.CustomDisplay(obj);
        end

        function displayEmptyObject(obj)
            displayEmptyObject@matlab.mixin.CustomDisplay(obj);
        end

        function displayScalarHandleToDeletedObject(obj)
            displayScalarHandleToDeletedObject@matlab.mixin.CustomDisplay(obj);
        end
    end

end

