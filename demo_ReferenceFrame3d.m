function demo_ReferenceFrame3d()

    hfig = figure;
    ax = axes('parent', hfig);
    hold(ax,'on');
    grid on
    xlim(ax, [-15 15]);
    ylim(ax, [-15 15]);
    zlim(ax, [-10 10]);
    view(ax, 45,45)
    axis(ax, 'vis3d');
    rotate3d(ax, 'on');
    on_exit = onCleanup(@() delete(hfig));

    % create all our coordinate systems up-front (relative to one another)
    world = ReferenceFrame3d(eye(3));
    base = ReferenceFrame3d(eye(3));
    first_arm = ReferenceFrame3d(eye(3), [10 0 0]); % fixed to the base
    second_arm = ReferenceFrame3d(eye(3), [2 0 3]); % in 1st arm's frame
    second_arm.rotate_eulerd(0, 15, 30);

    % plot all the objects in the axis
    frames = [world, base, first_arm, second_arm];
    frames.plot('Parent', ax, 'LineLength', 3);
    frames(end).draw_plane('Slice','xy');

    % draw the first arm (attached to the base, so we use the base's coordinate frame)
    line([0 first_arm.t(1)], [0 first_arm.t(2)], [0 first_arm.t(3)], ...
        'Parent', base.hgtransform(), ...
        'LineWidth', 2, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'FIRST_ARM');
    line([0 second_arm.t(1)], [0 second_arm.t(2)], [0 second_arm.t(3)], ...
        'Parent', first_arm.hgtransform(), ...
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
        % will automatically place it in the correct world frame location
        xc = -cos(10 * time);
        yc = sin(10 * time);
        h = plot3(second_arm.hgtransform(), xc, yc, 0, 'k.');

        % we'll also translate the coordinate out of the local frame and into the
        % world frame to confirm that the transform works correctly

        % concatenate to form a transformation sequence
        frames = [base, first_arm, second_arm]; 

        % transform from local of frame(end) to base of frame(1)
        base_pos = frames.local2base([xc yc 0]);

        % plot directly to the axis now (world frame) to prove equivalence w.r.t.
        % plotting in the local frame
        h(2) = plot3(ax, ... 
            base_pos(1), base_pos(2), base_pos(3), 'ro');

        drawnow
        delete(h);
    end
    
end
