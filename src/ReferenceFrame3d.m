classdef ReferenceFrame3d < matlab.mixin.Copyable
    
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

    %% matlab.mixin.Copyable
    methods
        function new = copyElement(this)
            new = copyElement@matlab.mixin.Copyable(this);

            % only copy graphics objects if something has been plotted
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                return
            end
            parent = this.h_transform.Parent;
            new.h_transform = copyobj(this.h_transform, parent);
            new.h_frame = copyobj(this.h_frame, parent);
        end
    end

    %% Dependent
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

    methods
        function [p, dist] = intersect_plane(this, observer, ray, opts)
            arguments
                this(1,1) ReferenceFrame3d
                observer(1,3) double
                ray(1,3) double
                opts.Slice(1,1) string = "xy"
                opts.Offset(1,1) double = 0
                opts.Debug(1,1) logical = true
            end

            % select the 2 basis vectors that define the plane
            switch opts.Slice
                case "xy"
                    a = this.x; b = this.y;
                case "xz"
                    a = this.x; b = this.z;
                case "yz"
                    a = this.y; b = this.z;
                case "yx"
                    a = this.y; b = this.x;
                case "zx"
                    a = this.z; b = this.x;
                case "zy"
                    a = this.z; b = this.y;
                otherwise
                    % validatestring
            end

            normal = cross(a, b);
            normal = normal ./ norm(normal);
            point = this.t + (opts.Offset * normal);

            denominator = dot(normal, ray);

            if abs(denominator) < 1e-6
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
    
    %% TODO
    methods
        function validate(this)
            assert(det(this.R) == 1, ...
                'Expected determinant of R to be 1; instead got %f', det(this.R));
        end

        function new = translate(this, dx)
            arguments
                this(1,1) ReferenceFrame3d
                dx(3,1) double
            end
            new = ReferenceFrame3d(this.T * makehgtform('translate', dx));
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
            new = ReferenceFrame3d(this.T * T);
        end

        function new = compose(this)
            arguments (Repeating)
                this(1,:) ReferenceFrame3d
            end
            this = [this{:}];
            T_new = this(end).T;
            for i = numel(this)-1:-1:1
                T_new = this(i).T * T_new;
            end
            new = ReferenceFrame3d(T_new);
        end

        function inverse = invert(this)
            R_inv = this.R'; % transpose = inverse for a DCM by definition
            t_inv = -R_inv * this.t;
            inverse = ReferenceFrame3d([R_inv, t_inv; 0 0 0 1]);
        end

        % overloads
        function new = mtimes(this, other)
            %MTIMES Matrix multiplication.
            new = ReferenceFrame3d(this.T * other.T);
        end

        function new = ctranspose(this)
            new = transpose(this); % complex types are not supported (' == .')
        end
        function new = transpose(this)
            T_new = this.T;
            T_new(1:3,1:3) = T_new(1:3,1:3).';
            new = ReferenceFrame3d(T_new);
        end

        % numeric representations
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
    end
    
    %% Constructors
    methods
        function this = ReferenceFrame3d(matrix, origin)
            if nargin == 0
                return
            end
            if nargin < 2
                origin = [0 0 0];
            end

            if isequal(size(matrix,[1 2]), [4 4])
                this.T = matrix;
            else
                matrix(4,4) = 1;
                this.T = matrix * makehgtform('translate', origin);
            end
        end

        function set(this, dcm, origin)
            %SET Set new reference frame parameters.

            switch class(dcm)
                case 'ReferenceFrame3d'
                    frame = dcm;
                    this.dcm = frame.dcm;
                    this.t = frame.t;
                case {'double','single'}
                    this.dcm = dcm;
                    this.t = origin;
                otherwise
                    validateattributes(dcm,{'single','double','ReferenceFrame3d'},{});
            end

            this.update_transform();
        end
    end

    %% Plotting
    methods
        function tform = hgtransform(this, parent)
            if nargin < 2
                parent = gca;
            end

            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                this.h_transform = hgtransform('Parent', parent);
            end

            tform = this.h_transform;
        end

        function plot(this)
            %PLOT Plot as a 3-d object.

            % DEBUG/TODO: remove
            figure
            ax = gca;
            xlim([-1 1]);
            ylim([-1 1]);
            zlim([-1 1]);
            grid on; box on;

            this.hgtransform(); % TODO: configurable parent
            this.update_transform();

            scale = 0.1 * max(diff(xlim(ax)), diff(ylim(ax)));

            % we may wish to plot other things to the reference frame transform,
            % so let's organize the basis vector data under its own group
            this.h_frame = hggroup('parent', this.h_transform);
            line(this.h_frame, ...
                'XData', 0, 'YData', 0, 'ZData', 0, ...
                'Color', 'k', ...
                'Marker', '.', ...
                'MarkerSize', 12, ...
                'Hittest','off','PickableParts','none',...
                'Clipping', 'off');
            line(this.h_frame, ...
                [0 scale], [0 0], [0 0], ...
                'Color', 'r', ...
                'LineWidth', 1, ...
                'Clipping', 'off');
            line(this.h_frame, ....
                [0 0], [0 scale], [0 0], ...
                'Color', 'g', ...
                'LineWidth', 1, ...
                'Clipping', 'off');
            line(this.h_frame, ...
                [0 0], [0 0], [0 scale], ...
                'Color', 'b', ...
                'LineWidth', 1, ...
                'Clipping', 'off');
        end

        function update_transform(this)
            %UPDATE_TRANSFORM Re-orient based on the object state.
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                return
            end

            this.h_transform.Matrix = this.T;
        end
    end

end

