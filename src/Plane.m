classdef Plane < ReferenceFrame3d
    %PLANE A 3-dimensional plane.
    
    % graphics
    properties (Access = protected)
        h_plane
    end

    %% Constructor
    methods
        function this = Plane(varargin)
            narginchk(0,2);

            switch nargin
                case 0 % default constructor
                    this.T = eye(4);
                case 1 % convert from ReferenceFrame3d
                    this.T = varargin{1}.T;
                case 2 % create from point & normal vector
                    point = varargin{1};
                    normal = varargin{2};

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
                    this.T = T;
            end
        end
    end

    %% Overloads
    methods
        function plot(this, opts)
            arguments
                this(1,1) ReferenceFrame3d
                opts.Size(1,2) double = [4 4] % [x y]
                opts.GridLineSpacing(1,2) double = [nan nan] % [x y]
                opts.DisplayStyle(1,1) string = "origin"
            end

            plot@ReferenceFrame3d(this);

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

end

