classdef Plane < ReferenceFrame3d
    %PLANE A 3-dimensional plane.
    
    properties (Dependent)
        normal(1,3) double % the normal vector that defines orientation
    end

    % graphics
    properties (Access = protected)
        h_plane
    end

    %% Plotting
    methods
        function plot(this)
            plot@ReferenceFrame3d(this)

            ax = gca;
            xlen = diff(xlim(ax));
            ylen = diff(ylim(ax));

            N = 11;
            grid = linspace(0, 1, N);
            [xdata, ydata] = meshgrid(grid * xlen, grid * ylen);

            % always create the plane at the origin at +Z, then use
            % the plane's parameters to construct a rotation to apply
            % to the hggtransform
            this.h_plane = hggroup(this.h_transform);
            surface(...
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
                'Parent', this.h_plane);
        end
    end

    %% Dependent
    methods
        function v = get.normal(this)
            v = this.dcm(:,3);
        end
    end

end

