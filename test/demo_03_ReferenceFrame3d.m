function demo_03_ReferenceFrame3d()

    %% Initialize reference frames
    
    % we want our plot to appear with origin = (0,0,0) at our local reference point with
    % basis vectors aligned to ENU.  since we are basically just saying that E=[1 0 0],
    % N=[0 1 0], and U=[0 0 1], this is just the identity transform (until we add more frames)
    axis_enu = ReferenceFrame3d();
    axis_enu.name = 'ENU';

    % to give our identity transform (above) meaning, we need to define how to go from ENU 
    % to the ECEF frame. so we need to describe where the ECEF frame is with respect to (FROM) 
    % the ENU frame (hence the call to inv()).  this ENU frame is fixed to the Earth.
    refpoint = [39.22 -121.815 1000];
    ecef = inv(ReferenceFrame3d.ecef2enu(refpoint, "degrees"));
    ecef.name = 'ECEF';

    % then describe where the local-level NED frame is with respect to ECEF (this is the
    % local-level frame attached to our aircraft and will change at every timestep)
    ned = ReferenceFrame3d.ecef2ned([37, -119, 1e3], "degrees");
    ned.name = 'NED';

    % and where the body frame is with respect to NED (it's co-located with NED and we'll
    % initialize the orientation to be 30-degrees yaw, 10-degrees pitch)
    body = ReferenceFrame3d.from_euler([0 0 0], [0 0 0], Units="degrees");
    body.name = 'BODY';

    %% Set the reference frame hierarchy with method hgtransform()

    % create the figure
    hfig = figure('units','normalized','position',[0.05 0.05 0.9 0.85]);
    ax = axes('parent', hfig);
    hold(ax, 'on');

    % the relationships are defined by the array order
    frames = [axis_enu, ecef, ned, body];
    frames.hgtransform(ax);

    % HYPOTHETICAL:
    % to create a new parent-child relationship, just call hgtransform() again. e.g.:
    %
    %   detector = ReferenceFrame3d(E2D, pos_ecef);
    %   detector.hgtransform(ecef);
    %
    % this would result in a hierarchy where ECEF has 2 children, i.e.:
    %
    %       AXIS_ENU --> ECEF --> NED --> BODY
    %                    ECEF --> DETECTOR
    %
    % and of course, you can add as many more relationships as you'd like

    %% Draw some terrain using included map data
    
    map = load('rf3d_demo_map.mat');

    % convert the lat-lon-alt terrain data to cartesian ECEF
    [lon, lat] = meshgrid(map.lon, map.lat);
    [x,y,z] = helper_llad2ecef(lat(:), lon(:), single(map.h(:)));
    x = reshape(x, size(map.h));
    y = reshape(y, size(map.h));
    z = reshape(z, size(map.h));

    % plot the terrain data parented to the ECEF transform
    surf(x, y, z, single(map.h), ...
        'Parent', ecef.hgtransform(), ...
        'LineStyle', 'none', ...
        'Clipping','off');

    % axes are going to be highly distorted so we'll need to specify 
    axis_enu.plot('LineLength', [1e4 1e4 1e3], 'Arrowheads', false, 'TextLabels', true);
    ned.plot('LineLength', [1e4 1e4 1e3], 'Arrowheads', false, 'TextLabels', true);
    body.plot('LineLength', [1e4 1e4 1e3], 'Arrowheads', false, 'TextLabels', true);

    material dull
    camlight
    view(-140, 50)
    axis(ax, 'off');
    rotate3d(ax, 'on');

    b = 1852 * 30;
    xlim(ax, [-1 1]*b);
    ylim(ax, [-1 1]*b);
    zlim(ax, [-500 5000]);

    %% Draw a trajectory over the terrain

    lat = linspace(38.07, 39.59, 1000);
    lon = linspace(-122.07, -121.26, 1000);
    alt = 2000;
    [x,y,z] = helper_llad2ecef(lat, lon, alt);

    h = plot3(x,y,z,'.-',...
        'Parent', ecef.hgtransform(), ...
        'LineWidth', 1, ...
        'Clipping','off');
    text(x(500), y(500), z(500), "  trajectory data", ...
        'Parent', ecef.hgtransform(), ...
        'Color', h.Color, ...
        'Clipping', 'off');

end

function [x,y,z] = helper_llad2ecef(lat, lon, alt)
    r = 6378137;
    ecc = 0.00669437999014;

    rc = r./sqrt(1.0 - ecc*sind(lat).^2);

    x = (rc + alt).*cosd(lat).*cosd(lon);
    y = (rc + alt).*cosd(lat).*sind(lon);
    z = (rc + alt - ecc*rc).*sind(lat);
end
