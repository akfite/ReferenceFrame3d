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
                opts.GridSpacing(1,2) uint32 = opts.Size % grid density for display [x y]
            end

            plot@ReferenceFrame3d(this)

            ax = gca;
            xlen = diff(xlim(ax));
            ylen = diff(ylim(ax));

            N = 11;
            grid = linspace(0, 1, N);
            [xdata, ydata] = meshgrid(grid * xlen, grid * ylen);

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
                opts.CoplanarTolerance(1,1) double = 1e-6
                opts.Debug(1,1) logical = true
            end

            [p, dist] = intersect_plane@ReferenceFrame3d(this, observer, ray, ...
                'Slice', 'xy', ...
                'Offset', 0, ...
                'CoplanarTolerance', opts.CoplanarTolerance,...
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

