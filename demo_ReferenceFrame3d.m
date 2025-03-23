function demo_ReferenceFrame3d()

    hfig = figure;
    ax = axes('parent', hfig);
    hold(ax,'on');
    grid on
    xlim(ax, [-15 15]);
    ylim(ax, [-15 15]);
    zlim(ax, [-15 15]);
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
    line([0 10], [0 0], [0 0], ...
        'Parent', base.get_or_create_hgtransform(), ...
        'LineWidth', 3, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'FIRST_ARM');
    first_arm = ReferenceFrame3d(eye(3), [10 0 0]); % track with a RF3d object
    first_arm.plot('Parent', base.get_or_create_hgtransform(), ...
        'LineLength', 3);

    % the second arm is mounted to the first, pitched at a 45 degree angle
    % and will also rotate around the z axis.  we'll define the coordinate frame
    % first and then use internal methods to figure out where the parts are in
    % world coordinates
    % TODO


    clock = tic;
    time = toc(clock);

    while true
        if ~isvalid(hfig)
            break
        end

        elapsed = toc(clock) - time;
        time = toc(clock);

        % base frame rotates at 30 deg/s around the z axis
        yaw = 30 * elapsed;
        base.rotate_euler(0, 0, yaw);
        base.update_hgtransform(); % TODO: handle this via listeners

        % an attachment on the arm rotates in the opposite direction at 90 deg/s
        roll = -300 * elapsed;
        first_arm.rotate_euler(roll, 0, 0);
        first_arm.update_hgtransform();

        drawnow
    end
    
end
