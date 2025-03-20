classdef ReferenceFrame3d < handle
    
    properties (SetAccess = private)
        R(4,4) double = eye(4) % homogeneous transform (rotation & translation)
    end

    properties (Dependent)
        x(1,3) double
        y(1,3) double
        z(1,3) double
        
        origin(1,3) double
    end

    % graphics
    properties (Access = protected)
        h_transform
        h_frame
    end

    %% Dependent
    methods
        function v = get.x(this)
            v = this.R(1:3,1).';
        end
        function v = get.y(this)
            v = this.R(1:3,2).';
        end
        function v = get.z(this)
            v = this.R(1:3,3).';
        end
        function v = get.origin(this)
            v = this.R(1:3,4).';
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
            point = this.origin + (opts.Offset * normal);

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
    
    %% Constructors
    methods
        function this = ReferenceFrame3d(R, origin)
            if nargin == 0
                return
            end
            if nargin < 2
                origin = [0 0 0];
            end

            if isequal(size(R,[1 2]), [4 4])
                this.R = R;
            else
                R(4,4) = 1;
                this.R = R * makehgtform("translate", origin);
            end
        end

        function set(this, dcm, origin)
            %SET Set new reference frame parameters.

            switch class(dcm)
                case 'ReferenceFrame3d'
                    frame = dcm;
                    this.dcm = frame.dcm;
                    this.origin = frame.origin;
                case {'double','single'}
                    this.dcm = dcm;
                    this.origin = origin;
                otherwise
                    validateattributes(dcm,{'single','double','ReferenceFrame3d'},{});
            end

            this.update_transform();
        end

        function tform_4x4 = homogeneous_transform(this)
            %HOMOGENEOUS_TRANSFORM Convert to 4x4 form (rotation & translation)
            tform_4x4 = this.dcm; % 3x3
            tform_4x4(4,4) = 1; % grow to 4x4
            tform_4x4 = tform_4x4 * makehgtform('translate', this.origin);
        end

        function C = mtimes(A, B)
            %MTIMES Overload for matrix multiplication.
            if isa(B,'ReferenceFrame3d')
                B_matrix = B.homogeneous_transform();
            elseif isnumeric(B)
                % convert numeric to 4x4 (assume no translation when 3x3)
                if size(B,1) == 3 && size(B,2) == 3
                    B_matrix(1:3,1:3) = B;
                elseif size(B,1) == 4 && size(B,2) == 4
                    B_matrix = B;
                else
                    error('Expected B to be 3xN or 4xN when numeric.');
                end
            end

            C_matrix = A.homogeneous_transform() * B_matrix;
            C = ReferenceFrame3d(C_matrix(1:3,1:3), C_matrix(1:3,4));
        end

        function new = ctranspose(this)
            R_new = this.R;
            R_new(1:3,1:3) = R_new(1:3,1:3)';
            new = ReferenceFrame3d(R_new);
        end

        function new = transpose(this)
            R_new = this.R;
            R_new(1:3,1:3) = R_new(1:3,1:3).';
            new = ReferenceFrame3d(R_new);
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

            this.h_transform.Matrix = homogeneous_transform(this);
        end
    end

end

