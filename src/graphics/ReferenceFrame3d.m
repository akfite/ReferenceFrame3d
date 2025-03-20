classdef ReferenceFrame3d < handle
    
    properties (SetAccess = private)
        dcm(3,3) double % orientation 
        origin(1,3) double
    end

    % graphics
    properties (Access = private)
        h_transform
        h_frame
    end
    
    %% Constructors
    methods
        function this = ReferenceFrame3d(dcm, origin)
            arguments
                dcm(3,3) double {mustBeReal} = eye(3)
                origin(1,3) double {mustBeReal} = [0 0 0]
            end

            this.dcm = dcm;
            this.origin = origin;
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
                    error('Expected B to be 3x3 or 4x4 when numeric.');
                end
            end

            C_matrix = A.homogeneous_transform() * B_matrix;
            C = ReferenceFrame3d(C_matrix(1:3,1:3), C_matrix(1:3,4));
        end

        function new = ctranspose(this)
            new = ReferenceFrame3d(this.dcm.', this.origin);
        end

        function new = transpose(this)
            new = ReferenceFrame3d(this.dcm', this.origin);
        end
    end

    %% Plotting
    methods
        function plot(this)
            %PLOT Plot as a 3-d object.

            % DEBUG/TODO: remove
            figure
            ax = gca;
            xlim([-1 1]);
            ylim([-1 1]);
            zlim([-1 1]);
            grid on; box on;

            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                % draw from scratch
                this.h_transform = hgtransform('Parent', ax);
            end

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

