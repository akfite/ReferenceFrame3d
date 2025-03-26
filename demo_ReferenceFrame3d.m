function demo_ReferenceFrame3d()

    hfig = figure;
    ax = axes('parent', hfig);
    hold(ax,'on');
    grid on
    xlim(ax, [-15 15]);
    ylim(ax, [-15 15]);
    zlim(ax, [-15 15]);
    view(45,45)
    axis vis3d
    on_exit = onCleanup(@() delete(hfig));

    % draw the world (fixed) coordinate system first
    world = ReferenceFrame3d(eye(3));
    world.plot('Parent', ax, ...
        'LineLength', 3);

    % the base frame will rotate with respect to the world
    base = ReferenceFrame3d(eye(3));
    base.plot('Parent', world.get_or_create_hgtransform(), ...
        'LineLength', 3);
    
    % the first arm is fixed to the base and rotates with it
    pos = [10 0 0];
    first_arm = ReferenceFrame3d(eye(3), pos); % track with a RF3d object
    first_arm.plot(...
        'Parent', base.get_or_create_hgtransform(), ...
        'LineLength', 1);
    % draw the first arm (attached to the base, so we use the base's coordinate frame)
    line([0 pos(1)], [0 pos(2)], [0 pos(3)], ...
        'Parent', base.get_or_create_hgtransform(), ...
        'LineWidth', 2, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'FIRST_ARM');

    % the second arm is mounted to the first (so we define its position relative to
    % the first arm, i.e. use the first arm's frame)
    pos = [2 0 3];
    second_arm = Plane(ReferenceFrame3d(eye(3), pos)); % in 1st arm's frame
    second_arm.rotate_eulerd(0, 15, 30);
    second_arm.plot('Parent', first_arm.get_or_create_hgtransform());
    line([0 pos(1)], [0 pos(2)], [0 pos(3)], ...
        'Parent', first_arm.get_or_create_hgtransform(), ...
        'LineWidth', 2, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'SECOND_ARM');

    clock = tic;
    time = toc(clock);

    warnstate = warning('off', 'MATLAB:hg:DiceyTransformMatrix');
    on_exit(end+1) = onCleanup(@() warning(warnstate));

    while true
        if ~isvalid(hfig)
            break
        end

        elapsed = toc(clock) - time;
        time = toc(clock);

        % base frame rotates at 30 deg/s around the z axis
        yaw = 30 * elapsed;
        base.rotate_eulerd(0, 0, yaw);

        % an attachment on the arm rotates in the opposite direction at 90 deg/s
        yaw = -180 * elapsed;
        first_arm.rotate_eulerd(0, 0, yaw);

        % now, as we rotate, let's animate a circular path in the second arm's frame.
        % we'll observe how we can plot directly in local coordinates, and the plot
        % will handle displaying it in the correct global location
        xc = cos(10 * time);
        yc = sin(10 * time);
        h = plot3(second_arm.get_or_create_hgtransform(), xc, yc, 0, 'k.');

        % we'll also translate the coordinate out of the local frame and into the
        % world frame to confirm that the transform works correctly
        frames = [base, first_arm, second_arm]; % concatenate to form a transform sequence
        base_pos = frames.local2base([xc yc 0]); % frames(end) transformed to frames(1)
        h(2) = plot3(ax, ... % notice the parent is the AXIS now (world frame)
            base_pos(1), base_pos(2), base_pos(3), 'ro');

        drawnow
        delete(h);
    end
    
end
