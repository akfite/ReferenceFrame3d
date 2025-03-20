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
    
    %% General
    methods
        function this = ReferenceFrame3d(dcm, origin)

            arguments
                dcm(3,3) double {mustBeReal} = eye(3)
                origin(1,3) double {mustBeReal} = [0 0 0]
            end

            this.dcm = dcm;
            this.origin = origin;
        end

        function update(this, dcm, origin)
            %UPDATE Update the reference frame's parameters.

            arguments
                this(1,1) ReferenceFrame3d
                dcm(3,3) double {mustBeReal}
                origin(1,3) double {mustBeReal} = this.origin
            end

            this.dcm = dcm;
            this.origin = origin;

            this.update_transform();
        end
    end

    %% Plotting
    methods
        function plot(this, ax)
            %PLOT Plot as a 3-d object.

            ax = gca;

            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                % draw from scratch
                this.h_transform = hgtransform('Parent', ax);
            end

            this.update_transform();

            % TODO: figure out how to plot at a reasonable size
            scale = 0.1;

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
                'LineWidth', 2, ...
                'Clipping', 'off');
            line(this.h_frame, ....
                [0 0], [0 scale], [0 0], ...
                'Color', 'g', ...
                'LineWidth', 2, ...
                'Clipping', 'off');
            line(this.h_frame, ...
                [0 0], [0 0], [0 scale], ...
                'Color', 'b', ...
                'LineWidth', 2, ...
                'Clipping', 'off');
        end

        function update_transform(this)
            %UPDATE_TRANSFORM Re-orient based on the object state.
            if isempty(this.h_transform) || ~isvalid(this.h_transform)
                return
            end

            tform = this.dcm;
            tform(4,4) = 1; % make 4x4
            tform = tform * makehgtform('translate', this.origin);

            this.h_transform.Matrix = tform;
        end
    end

end

