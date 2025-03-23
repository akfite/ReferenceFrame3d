classdef ReferenceFrame3d < matlab.mixin.Copyable & matlab.mixin.CustomDisplay
    
    properties (SetAccess = private)
        T(4,4) double {mustBeReal} = eye(4) % homogeneous transform (rotation & translation)
    end

    properties (Dependent)
        R(3,3) double % rotation submatrix
        x(3,1) double % x basis (1st col)
        y(3,1) double % y basis (2nd col)
        z(3,1) double % z basis (3rd col)
        t(3,1) double % translation (origin of reference frame)
    end

    % graphics
    properties (Access = protected)
        h_transform
        h_frame
    end

    %% Construction
    methods
        function this = ReferenceFrame3d(matrix, origin)
            if nargin == 0
                return
            end
            if nargin < 2
                origin = [0 0 0];
            end

            if isequal(size(matrix,[1 2]), [4 4])
                T = matrix;
            else
                matrix(4,4) = 1;
                T = matrix * makehgtform('translate', origin);
            end

            ReferenceFrame3d.validate_transform(T);
            this.T = T;
        end
    end

    methods (Static)
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
    methods
        function base_vec = local2base(this, local_vec)
            arguments
                this(:,1) ReferenceFrame3d
                local_vec(:,3)
            end

            if isscalar(this)
                T = this.T;
            else
                T = as_matrix(compose(this)); % combine rotations
            end

            local_vec(:,4) = 1; % to homogeneous coordinates

            % create a dimension deleter (delete 4th dim of output)
            H = eye(4);
            H(4,:) = [];

            base_vec = (H * T * local_vec')'; % transform
        end

        function local_vec = base2local(this, base_vec)
            arguments
                this(:,1) ReferenceFrame3d
                base_vec(:,3)
            end

            if isscalar(this)
                T = as_matrix(inv(this));
            else
                T = as_matrix(inv(compose(this))); % combine rotations
            end

            base_vec(:,4) = 1; % to homogeneous coordinates

            % create a dimension deleter (delete 4th dim of output)
            H = eye(4);
            H(4,:) = [];

            local_vec = (H * T * base_vec')'; % transform
        end

        function new = translate(this, dx)
            arguments
                this(1,1) ReferenceFrame3d
                dx(3,1) double
            end
            new = ReferenceFrame3d(this.T * makehgtform('translate', dx));
            if nargout == 0, this.T = T_new; end
        end

        function new = reposition(this, new_pos)
            arguments
                this(1,1) ReferenceFrame3d
                new_pos(3,1) double
            end
            T_new = this.T;
            T_new(1:3,4) = new_pos;
            new = ReferenceFrame3d(T_new);
            if nargout == 0, this.T = T_new; end
        end
        
        function new = rotate(this, dcm)
            arguments
                this(1,1) ReferenceFrame3d
                dcm(3,3) double
            end
            T = dcm; %#ok<*PROPLC>
            T(4,4) = 1; % 3x3 -> 4x4 with no translation
            T_new = this.T * T;
            new = ReferenceFrame3d(T_new);
            if nargout == 0, this.T = T_new; end
        end

        function new = rotate_euler(this, roll, pitch, yaw)
            T = makehgtform('xrotate', roll * pi/180) ...
                * makehgtform('yrotate', pitch * pi/180) ...
                * makehgtform('zrotate', yaw * pi/180);
            new = ReferenceFrame3d(T * this.T);
            if nargout == 0, this.T = new.T; end
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

        function inverse = inv(this)
            R_inv = this.R'; % transpose = inverse for a DCM by definition
            t_inv = -R_inv * this.t;
            T_new = [R_inv, t_inv; 0 0 0 1];
            inverse = ReferenceFrame3d(T_new);
            if nargout == 0, this.T = T_new; end
        end

        function T = as_matrix(this)
            arguments
                this(1,1) ReferenceFrame3d
            end
            T = this.T;
        end

        function as_quaternion()
        end

        function [roll, pitch, yaw] = as_euler()
        end

        function [roll, pitch, yaw] = angle_between(A, B)
        end

        function [p, dist] = intersect_plane(this, observer, ray, opts)
            arguments
                this(1,1) ReferenceFrame3d
                observer(1,3) double
                ray(1,3) double
                opts.Slice(1,2) char = 'xy'
                opts.Offset(1,1) double = 0
                opts.CoplanarTolerance(1,1) double = 1e-6
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

            if abs(denominator) < opts.CoplanarTolerance
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

        function new = ctranspose(this)
            new = transpose(this); % complex types are not supported (' == .')
            if nargout == 0, this.T = T_new; end
        end

        function new = transpose(this)
            T_new = this.T;
            T_new(1:3,1:3) = T_new(1:3,1:3).';
            new = ReferenceFrame3d(T_new);
            if nargout == 0, this.T = T_new; end
        end
    end

    %% Graphics
    methods
        function show(this)
            % draw everything in a new, dedicated figure

            % DEBUG/TODO: remove
            hfig = figure;
            ax = axes('parent', hfig);
            xlim(ax, [-1 1]); % TODO: consider frame origin
            ylim(ax, [-1 1]);
            zlim(ax, [-1 1]);
            grid on; box on;
            this.plot('Colors','k', LineStyle=':');
        end

        function plot(this, opts)
            %PLOT Plot as a 3-d object.
            arguments
                this(1,1) ReferenceFrame3d
                opts.Parent = []
                opts.Colors(1,3) char = 'rgb' % for basis xyz
                opts.LineWidth(1,3) double = 1
                opts.LineStyle(1,:) char = '-'
                opts.LineLength(1,3) double = 1
                opts.EnableArrowheads(1,3) logical = 1
            end

            tform = this.get_or_create_hgtransform(opts.Parent);
            this.update_hgtransform(); % TODO: this should happen via listeners

            if ~isempty(this.h_frame) ...
                    && isvalid(this.h_frame) ...
                    && isequal(this.h_frame.Parent, tform)
                return % data is already plotted
            end
            
            ax = ancestor(tform, 'axes');
            assert(~isempty(ax), ...
                'The ancestor axis to the hgtransform is empty.');
            sz = opts.LineLength;

            % we may wish to plot other things to the reference frame transform,
            % so let's organize the basis vector data under its own group
            this.h_frame = hggroup('parent', tform, 'tag', 'RF3D_BASIS_VECTORS');

            if all(opts.LineWidth == opts.LineWidth(1)) && all(opts.Colors == opts.Colors(1))
                % interleave NaNs to plot all basis vectors as a single object
                xdata = [0 sz(1) NaN; 0 0 NaN; 0 0 NaN]';
                ydata = [0 0 NaN; 0 sz(2) NaN; 0 0 NaN]';
                zdata = [0 0 NaN; 0 0 NaN; 0 sz(3) NaN]';
                line(this.h_frame, ...
                    xdata(:), ydata(:), zdata(:), ...
                    'Color', opts.Colors(1), ...
                    'LineWidth', opts.LineWidth(1), ...
                    'LineStyle', opts.LineStyle, ...
                    'Clipping', 'off');
            else
                line(this.h_frame, ...
                    [0 sz(1)], [0 0], [0 0], ...
                    'Color', opts.Colors(1), ...
                    'LineWidth', opts.LineWidth(1), ...
                    'LineStyle', opts.LineStyle, ...
                    'Clipping', 'off');
                line(this.h_frame, ....
                    [0 0], [0 sz(2)], [0 0], ...
                    'Color', opts.Colors(2), ...
                    'LineWidth', opts.LineWidth(2), ...
                    'LineStyle', opts.LineStyle, ...
                    'Clipping', 'off');
                line(this.h_frame, ...
                    [0 0], [0 0], [0 sz(3)], ...
                    'Color', opts.Colors(3), ...
                    'LineWidth', opts.LineWidth(3), ...
                    'LineStyle', opts.LineStyle, ...
                    'Clipping', 'off');
            end
            for i = 1:3
                if opts.EnableArrowheads(i)
                    plot_arrowhead(this.h_frame, sz(i), opts.Colors(i), i);
                end
            end
            line(this.h_frame, ...
                'XData', 0, 'YData', 0, 'ZData', 0, ...
                'Color', 'k', ...
                'Marker', '.', ...
                'MarkerSize', 12, ...
                'Hittest','off','PickableParts','none',...
                'Clipping', 'off');

            function plot_arrowhead(parent, base_length, color, basis)
                % define the cone in the N-frame
                m = 20; n = 2;
                theta = linspace(0, 2*pi, m);
                R = linspace(0, 0.2*base_length, n);
                [T, R] = meshgrid(theta, R);
    
                % to cartesian coordinates
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

        function tform = get_or_create_hgtransform(this, parent)
            %GET_HGTRANSFORM Get the graphics transform paired to this object.

            arguments
                this(1,1) ReferenceFrame3d
                parent = []
            end

            if isa(parent, 'ReferenceFrame3d')
                assert(~isempty(parent.h_transform) && isvalid(parent.h_transform), ...
                    'Invalid parent (no transform has been created)');
                parent = parent.h_transform;
            end

            % consider the current transform invalid if the parent has changed
            if ~isempty(parent) && isvalid(parent) ...
                    && ~isempty(this.h_transform) ...
                    && isvalid(this.h_transform) ...
                    && ~isequal(this.h_transform.Parent, parent)
                delete(this.h_transform);
            end

            % create new axes only if we don't have a parent to target
            % AND there's no valid transform already
            if (isempty(parent) || ~isvalid(parent)) ...
                    && (isempty(this.h_transform) || ~isvalid(this.h_transform))
                parent = gca();
            end

            % create a new transform
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                this.h_transform = hgtransform('Parent', parent);
            end

            tform = this.h_transform;
        end

        function update_hgtransform(this)
            %UPDATE_TRANSFORM Update the graphics transform to match the object state.
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                return
            end
            this.h_transform.Matrix = this.T;
        end

        function clear(this)
            %CLEAR Delete all graphics objects.
            for i = 1:numel(this)
                delete(this(i).h_transform);
            end
        end
    end

    %% Dependent property accessors
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
            copied.h_frame = copyobj(this.h_frame, parent);
        end
    end

    %% matlab.mixin.CustomDisplay
    methods (Access = protected)
    end

end

