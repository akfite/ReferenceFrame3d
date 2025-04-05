function demo_02_ReferenceFrame3d()

    % The CAD file for this demo was sourced (and modified) from here:
    % https://grabcad.com/library/f-22-raptor-2

    assert(~isempty(which('rf3d_demo_f22.mat')), ...
        ['Expected to find the demo file "rf3d_demo_f22.mat" on the path.  ' ...
        'It should be in the "./assets" folder']);

    hfig = figure;
    ax = axes('parent', hfig);
    hold(ax,'on');
    grid(ax,'on');
    view(ax, 45, 20)
    axis(ax, 'vis3d');
    axis(ax, 'equal');
    rotate3d(ax, 'on');
    % axis(ax, 'off');
    xlim(ax, [-5 5]);
    ylim(ax, [-5 5]);
    zlim(ax, [-2 2]);
    on_exit = onCleanup(@() delete(hfig));

    % create all our coordinate systems up-front (relative to one another)
    base = ReferenceFrame3d(eye(3));
    model = ReferenceFrame3d(eye(3), [5 0 0]);

    % create the hgtransforms and nothing else
    frames = [base, model];
    frames.hgtransform(ax);

    % attach the f22 model to the body frame -- note that for this to work right,
    % the vertices need to be defined with "forward" = x, "port" = y, "up" = z
    load('rf3d_demo_f22.mat'); %#ok<*LOAD>
    patch('faces', fv.faces, 'vertices', fv.vertices, ...
        'Parent', model.hgtransform(), ...
        'FaceColor', 0.5*[1 1 1], ...
        'LineStyle', 'none', ...
        'Clipping','off');
    camlight(ax);

    % make it appear in a banked turn
    model.rotate_euler([-90, -5, 30]);

    % and for debugging, we can always add back the basis vectors if we want!
    % frames.plot('LineLength',1.5,'TextLabels',true); % uncomment

    %% animation loop (make the f22 orbit in a circle)
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

        drawnow
    end
    
end
