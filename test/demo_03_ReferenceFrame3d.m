function demo_03_ReferenceFrame3d()

    refpoint = [35 -117 0];

    % create the figure
    hfig = figure('units','normalized','position',[0.05 0.05 0.9 0.85]);
    tl = tiledlayout(hfig, 1, 2);
    ax(1) = nexttile(tl);
    ax(2) = nexttile(tl);
    hold(ax, 'on');

    %% Initialize reference frames
    
    % we want our plot to appear with origin = (0,0,0) at our local reference point with
    % basis vectors aligned to ENU.  since we are basically just saying that E=[1 0 0],
    % N=[0 1 0], and U=[0 0 1], this is just the identity transform (until we add more frames)
    axis_enu = ReferenceFrame3d();
    axis_enu.name = 'ENU';

    % to give our identity transform (above) meaning, we need to define how to go from ENU 
    % to the ECEF frame. so we need to describe where the ECEF frame is with respect to (FROM) 
    % the ENU frame (hence the call to inv()).  this ENU frame is fixed to the Earth.
    ecef = inv(ReferenceFrame3d.ecef2enu(refpoint, "degrees"));
    ecef.name = 'ECEF';

    % then describe where the local-level NED frame is with respect to ECEF (this is the
    % local-level frame attached to our aircraft and will change at every timestep)
    ned = ReferenceFrame3d.ecef2ned([35, -117, 1e3], "degrees");
    ned.name = 'NED';

    % and where the body frame is with respect to NED (it's co-located with NED and we'll
    % initialize the orientation to be 30-degrees yaw, 10-degrees pitch)
    body = ReferenceFrame3d.from_euler([30 10 0], [0 0 0], Units="degrees");
    body.name = 'BODY';

    %% Set the reference frame hierarchy with method hgtransform()

    % the relationships are defined by the array order
    frames = [axis_enu, ecef, ned, body];
    frames.hgtransform(ax(1));

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

    %% In the left-side axis, show the frames with respect to the Earth

    % plot a super simple sphere to represent the Earth
    r = 6378137; % earth equatorial radius, meters
    [x,y,z] = sphere(50);
    x = x * r;
    y = y * r;
    z = z * r;

    % our sphere is defined in the ECEF frame, so parent to the ECEF frame
    surf(x,y,z,'FaceAlpha',0.5,'Parent',ecef.hgtransform(), 'clipping','off');

    % display all our frames in the axis
    plot(frames, 'LineLength', r/3, 'TextLabels',true, 'Parent', ax(1), 'Detach', true);
    
    n = 5e6;
    xlim(ax(1),n*[-1 1]);
    ylim(ax(1),n*[-1 1]);
    zlim(ax(1),n*[-1 1]);

    axis(ax(1),'vis3d');
    axis(ax(1), 'off');
    rotate3d(ax(1),'on');
    view(ax(1), -45, 25)

    %% In the right-side axis

end