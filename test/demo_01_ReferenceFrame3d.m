function demo_01_ReferenceFrame3d()

    hfig = figure;
    ax = axes('parent', hfig);
    hold(ax,'on');
    grid on
    view(ax, 45, 20)
    axis(ax, 'vis3d');
    axis(ax, 'equal');
    rotate3d(ax, 'on');
    axis(ax, 'off');
    xlim(ax, [-15 15]);
    ylim(ax, [-15 15]);
    zlim(ax, [-10 10]);
    on_exit = onCleanup(@() delete(hfig));

    % create all our coordinate systems up-front (relative to one another)
    world = ReferenceFrame3d(eye(3));
    base = ReferenceFrame3d(eye(3));
    first_arm = ReferenceFrame3d(eye(3), [10 0 0]); % fixed to the base
    second_arm = ReferenceFrame3d(eye(3), [2 0 3]); % in 1st arm's frame
    second_arm.rotate_euler([30, 15, 0]);

    % plot all the objects in the axis
    frames = [world, base, first_arm, second_arm];
    frames.plot('Parent', ax, 'LineLength', 3);

    % if you want to plot without the basis vectors, call hgtransform()
    % instead of plot() to only establish the hgtransform object hierarchy, e.g.:
    % frames.hgtransform(ax);

    % add our own plots to supplement the basis vectors already drawn
    line([0 first_arm.origin(1)], [0 first_arm.origin(2)], [0 first_arm.origin(3)], ...
        'Parent', base.hgtransform(), ...
        'LineWidth', 2, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'FIRST_ARM');
    line([0 second_arm.origin(1)], [0 second_arm.origin(2)], [0 second_arm.origin(3)], ...
        'Parent', first_arm.hgtransform(), ...
        'LineWidth', 2, ...
        'Color', 'k', ...
        'Marker', '.', ...
        'Tag', 'SECOND_ARM');

    world.draw_plane(...
        'Clipping','on',...
        'Size',[50 50],...
        'GridLineSpacing',[5 5],...
        'FaceAlpha',0.05,...
        'EdgeAlpha',0.1);
    base.draw_plane(...
        'Clipping','on',...
        'Size',[50 50],...
        'GridLineSpacing',[5 5],...
        'FaceAlpha',0.05);
    second_arm.draw_plane('FaceColor','r');

    %% animation loop
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

        % base frame rotates at -30 deg/s around the z axis
        yaw = -30 * elapsed;
        base.rotate_euler([yaw, 0, 0]);

        % an attachment on the arm rotates in the opposite direction
        yaw = 180 * elapsed;
        first_arm.rotate_euler([yaw, 0, 0]);

        % now, as we rotate, let's animate a circular path in the second arm's frame.
        % we'll observe how we can plot directly in local coordinates, and the plot
        % will automatically place it in the correct world frame location
        xc = -cos(10 * time);
        yc = sin(10 * time);
        h = plot3(second_arm.hgtransform(), ... % note the parent object
            xc, yc, 0, 'ko', 'MarkerSize', 6);

        % now, just for example, let's plot that same rotating point but in the
        % world frame, using methods of ReferenceFrame3d

        % concatenate to form a transformation sequence
        frames = [base, first_arm, second_arm]; 

        % transform from local of frames(end) to base of frames(1)
        [wx, wy, wz] = frames.local2base(xc, yc, 0);
        % wxyz = frames.local2base([xc, yc, 0]); % could also call with Nx3 input/output

        % plot directly to the axis now (world frame) to prove equivalence w.r.t.
        % plotting in the local frame
        h(2) = plot3(ax, ... % note the parent object
            wx, wy, wz, 'ro', 'MarkerSize', 10);

        drawnow
        delete(h);
    end
    
end
