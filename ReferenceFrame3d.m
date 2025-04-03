classdef ReferenceFrame3d < matlab.mixin.Copyable ...
        & matlab.mixin.CustomDisplay ...
        & matlab.mixin.SetGet
    
    properties
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
        function this = ReferenceFrame3d(matrix, origin)
            %REFERENCEFRAME3D Constructor.
            arguments
                matrix = eye(4)
                origin(1,3) double = [0 0 0]
            end
            this.update(matrix, origin);
        end

        function this = update(this, rot, origin)
            %UPDATE Set the state of an existing object.
            arguments
                this(1,1) ReferenceFrame3d
                rot
                origin(1,3) double = [0 0 0]
            end

            sz = size(rot);

            if isscalar(rot)
                switch string(class(rot))
                    case "ReferenceFrame3d"
                        T = rot.T; % only transform; no handless
                    case "quaternion"
                        T = rotmat(quat,"frame");
                        T(4,4) = 1;
                    case "se3"
                        T = tform(rot);
                    case "so3"
                        T = tform(rot);
                        T(1:3,4) = origin;
                    otherwise
                        error('Unsupported scalar type %s', class(rot));
                end
            elseif isequal(sz, [4 4]) % homogeneous transform (4x4 matrix)
                T = rot;
            elseif isequal(sz, [3 3]) % DCM + origin
                rot(4,4) = 1;
                rot(1:3,4) = origin;
                T = rot;
            end

            this.T = T; % triggers set.T()
        end
    end

    methods (Static)
        function obj = from_point_normal(point, normal)
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
            obj = ReferenceFrame3d(T);
        end

        function obj = from_coplanar_vectors(v1, v2, origin)
            %FROM_COPLANAR_VECTORS Create a frame using a coplanar vector pair.
            arguments
                v1(1,3) double
                v2(1,3) double
                origin(1,3) double = [0 0 0]
            end

            x_basis = v1;
            z_basis = cross(v1, v2);
            y_basis = cross(z_basis, x_basis);

            dcm = [x_basis(:) y_basis(:) z_basis(:)];
            dcm = dcm ./ vecnorm(dcm, 2, 1);

            obj = ReferenceFrame3d(dcm, origin);
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

        function this = translate(this, dxyz)
            %TRANSLATE Shift the origin by an incremental amount.
            arguments
                this(1,1) ReferenceFrame3d
                dxyz(3,1) double {mustBeReal, mustBeFinite}
            end
            this.T(1:3,4) = this.T(1:3,4) + dxyz;
        end

        function this = reposition(this, new_pos)
            %REPOSITION Set a new origin.
            arguments
                this(1,1) ReferenceFrame3d
                new_pos(3,1) double {mustBeReal, mustBeFinite}
            end
            this.T(1:3,4) = new_pos;
        end
        
        function this = rotate(this, dcm)
            %ROTATE Rotate with a 3x3 Direction Cosine Matrix (DCM).
            arguments
                this(1,1) ReferenceFrame3d
                dcm(3,3) double
            end
            T = dcm; %#ok<*PROPLC>
            T(4,4) = 1; % 3x3 -> 4x4 with no translation
            this.T = this.T * T;
        end

        function this = rotate_euler(this, yaw, pitch, roll)
            %ROTATE_EULER Rotate with an euler sequence (zyx)
            cy = cos(yaw);
            sy = sin(yaw);
            cp = cos(pitch);
            sp = sin(pitch);
            cr = cos(roll);
            sr = sin(roll);
    
            T = [...
                          cy*cp,              sy*cp,      -sp,       0;
                -sy*cr+cy*sp*sr,     cy*cr+sy*sp*sr,    cp*sr,       0;
                 sy*sr+cy*sp*cr,    -cy*sr+sy*sp*cr,    cp*cr,       0
                              0,                  0,        0,       1];

            this.T = this.T * T;
        end

        function this = rotate_eulerd(this, yaw, pitch, roll)
            %ROTATE_EULERD Rotate with an euler sequence (zyx), in degrees
            this.rotate_euler(yaw * pi/180, pitch * pi/180, roll * pi/180);
        end

        function new = compose(this)
            %COMPOSE Multiply through a sequence of transforms T = T1*T2*T3...
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
            %INV Inverse of the transform.
            R_inv = this.R'; % transpose = inverse for a DCM by definition
            t_inv = -R_inv * this.origin;
            this.T = [R_inv, t_inv; 0 0 0 1];
        end
    end

    methods
        function [p, dist] = intersect_plane(this, observer, ray, opts)
            %INTERSECT_PLANE Fire a ray at a 2D slice of the reference frame.
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
            point = this.origin + (opts.Offset * normal);

            denominator = dot(normal, ray);

            if abs(denominator) < opts.Tol
                p = [nan nan nan];
                return
            end

            numerator = dot(normal(:), point(:) - observer(:));
            dist = numerator / denominator;
            p = observer + (dist * ray);

            if opts.Debug
                % create the figure & basis vectors
                ax = this.show();

                % try to size the plane in a way that makes sense for the problem
                sz = unique(abs(p(p ~= 0))*4);
                if isempty(sz), sz = [4 4]; end
                if numel(sz) > 2, sz = maxk(sz, 2)*4; end

                % draw the plane & intersecting line
                this.draw_plane('Slice', opts.Slice, ...
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
        function T = as_transform(this)
            %AS_TRANSFORM Express as a 4x4 homogeneous transform.
            arguments
                this(1,1) ReferenceFrame3d
            end
            T = this.T;
        end

        function R = as_dcm(this)
            %AS_DCM Express as a 3x3 pure rotation matrix.
            arguments
                this(1,1) ReferenceFrame3d
            end
            R = this.R;
        end

        function [yaw, pitch, roll] = as_euler(this)
            %AS_EULER Express as euler angles (zyx sequence).
            dcm = this.R;
            yaw = atan2(dcm(1,2,:), dcm(1,1,:));
            pitch = asin(-dcm(1,3,:));
            roll = atan2(dcm(2,3,:), dcm(3,3,:));
        end

        function [yaw, pitch, roll] = as_eulerd(this)
            %AS_EULERD Express as euler angles (zyx sequence), in degrees.
            [yaw, pitch, roll] = as_euler(this);
            yaw = yaw * 180/pi;
            pitch = pitch * 180/pi;
            roll = roll * 180/pi;
        end
    end

    %% MATLAB Toolbox Interoperability
    %
    %   These methods are subject to the availability of toolboxes on your system.

    methods
        function tform = se3(this)
            %SE3 Convert to an SE3 object.
            tform = se3(cat(3, this.T));
        end

        function rot = so3(this)
            %SO3 Convert to an SO3 object.
            rot = so3(cat(3, this.R));
        end

        function q = quaternion(this)
            %QUATERNION Convert to a quaternion object.
            q = quaternion(cat(3, this.R));
        end
    end

    %% Overloads
    methods
        function new = mtimes(this, other)
            %MTIMES Matrix multiplication.
            new = ReferenceFrame3d(this.T * other.T);
        end

        function this = ctranspose(this)
            %CTRANSPOSE Complex conjugate transpose (rotation component only).
            transpose(this);
        end

        function this = transpose(this)
            %TRANSPOSE Transpose (rotation component only).
            this.T(1:3,1:3) = this.T(1:3,1:3).';
        end
    end

    %% Graphics
    methods (Sealed)
        function ax = show(this)
            %SHOW Plot everything in a new, dedicated figure.
            hfig = figure;
            ax = axes('parent', hfig);
            grid(ax, 'on');
            box(ax, 'on');
            this.plot('parent',ax);
            hold(ax,'on');
            view(ax,45,30);
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
                opts.Disable(1,1) logical = false % delete and return early
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
                if opts.Disable, continue; end
                objs(i).h_plot_group = hggroup('Parent', parent, 'Tag', 'RF3D_BASIS_GROUP');
    
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
                        text(sz(1)*(j==1), sz(2)*(j==2), sz(3)*(j==3), ...
                            string([' ' basis_letter(j)]), ...
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
                m = 20; n = 2;
                theta = linspace(0, 2*pi, m);
                R = linspace(0, 0.2*base_length, n);
                [T, R] = meshgrid(theta, R);
    
                switch basis
                    case 1 % x
                        x = base_length - R;
                        y = R.*.2.*sin(T);
                        z = R.*.2.*cos(T);
                    case 2 % y
                        y = base_length - R;
                        z = R.*.2.*sin(T);
                        x = R.*.2.*cos(T);
                    case 3 % z
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
            for i = 1:numel(objs)
                if isempty(objs(i).h_transform) || ~isvalid(objs(i).h_transform)
                    continue
                end
                objs(i).h_transform.Matrix = objs(i).T;
            end
        end

        function clear(objs)
            %CLEAR Delete all graphics objects.
            for i = 1:numel(objs)
                delete(objs(i).h_transform);
                objs(i).h_transform = matlab.graphics.primitive.Transform.empty;
                objs(i).h_plot_group = matlab.graphics.primitive.Group.empty;
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

        function v = get.origin(this)
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
        function footer = getFooter(this)
            deg = char(176);

            if isscalar(this)
                footer = '';
                frame = this;
            else
                % don't display a custom footer for massive or multi-dim arrays
                if numel(this) > 100 || nnz(size(this) ~= 1) > 1
                    footer = ''; return
                end

                footer = '   This array represents a transformation sequence.';
                footer = sprintf('%s  When composed:\n\n', footer);

                frame = compose(this);
            end

            [y,p,r] = frame.as_eulerd();

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
                '       yaw   = %14.9f%s\n' ...
                '       pitch = %14.9f%s\n' ...
                '       roll  = %14.9f%s\n'], ...
                footer, ...
                frame.origin, round(y+eps,9), deg, round(p+eps,9), deg, round(r+eps,9), deg);
        end
    end

end

