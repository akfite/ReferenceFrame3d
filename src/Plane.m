classdef Plane < ReferenceFrame3d
    %PLANE A 3-dimensional plane.
    
    properties (Dependent)
        normal(1,3) double % the normal vector that defines orientation
    end

    % graphics
    properties (Access = protected)
        h_plane
    end

    %% Overloads
    methods
        function plot(this, opts)
            arguments
                this(1,1) ReferenceFrame3d
                opts.Size(1,2) double = [1 1] % [x y]
                opts.GridLineSpacing(1,2) double = [nan nan] % [x y]
                opts.DisplayStyle(1,1) string = "origin"
            end

            plot@ReferenceFrame3d(this)

            % calculate number of grid lines to draw in each dim
            if any(isnan(opts.GridLineSpacing))
                opts.GridLineSpacing = opts.Size;
            end

            grid_x = 0 : opts.GridLineSpacing(1) : opts.Size(1);
            grid_y = 0 : opts.GridLineSpacing(2) : opts.Size(2);
            if grid_x(end) ~= opts.Size(1)
                grid_x(end+1) = opts.Size(1);
            end
            if grid_y(end) ~= opts.Size(2)
                grid_y(end+1) = opts.Size(2);
            end

            [xdata, ydata] = meshgrid(grid_x, grid_y);

            % always create the plane at the origin at +Z
            this.h_plane = surface(...
                'XData', xdata, ...
                'YData', ydata, ...
                'ZData', zeros(size(xdata)), ...
                'FaceColor', 0.6*[1 1 1], ...
                'FaceAlpha', 0.2, ...
                'EdgeColor', 0.2*[1 1 1], ...
                'EdgeAlpha', 0.2, ...
                'LineStyle', '-', ...
                'Clipping', 'off', ...
                'HitTest','off',...
                'PickableParts','none',...
                'Parent', this.h_frame);
        end

        function [p, dist] = intersect_plane(this, observer, ray, opts)
            arguments
                this(1,1) Plane
                observer(1,3) double
                ray(1,3) double
                opts.Tol(1,1) double = 1e-6
                opts.Debug(1,1) logical = true
            end

            [p, dist] = intersect_plane@ReferenceFrame3d(this, observer, ray, ...
                'Slice', 'xy', ...
                'Offset', 0, ...
                'Tol', opts.Tol,...
                'Debug', opts.Debug);
        end
    end

    %% Dependent
    methods
        function v = get.normal(this)
            v = this.dcm(:,3);
        end
    end

end

