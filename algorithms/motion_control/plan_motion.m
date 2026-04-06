function [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars)

    if ~isfield(public_vars, 'initialized')
        public_vars.initialized = true;

        public_vars.path_id = 1;

        switch public_vars.path_id
            case 1
                x = linspace(1,9,80)';
                y = 1.5 * ones(size(x));
                public_vars.path = [x y];

            case 2
                phi = linspace(pi/2, 0, 120)';
                cx = 4;
                cy = 5.5;
                R = 3;
                public_vars.path = [cx + R*cos(phi), cy + R*sin(phi)];

            case 3
                y = linspace(1,9,120)';
                x = 5 + 0.8*sin(1.2*(y-1.0));
                public_vars.path = [x y];
        end

        public_vars.path_idx = 1;
    end

    pose = read_only_vars.mocap_pose;
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    path = public_vars.path;
    N = size(path,1);

    % konec trajektorie
    goal = path(end,:);
    if norm([x - goal(1), y - goal(2)]) < 0.2
        motion_vector = [0, 0];
        return;
    end

    % posun indexu dopředu jen pokud jsme dost blízko aktuálnímu bodu
    current_target = path(public_vars.path_idx,:);
    if norm([x - current_target(1), y - current_target(2)]) < 0.25
        public_vars.path_idx = min(public_vars.path_idx + 1, N);
    end

    % lookahead o pár bodů dopředu
    target_idx = min(public_vars.path_idx + 3, N);
    target = path(target_idx,:);

    dx = target(1) - x;
    dy = target(2) - y;

    target_angle = atan2(dy, dx);
    err = wrapToPi(target_angle - theta);

    % základní řízení
    v = 0.35 * max(0.2, cos(err));
    w = 1.2 * err;

    % převod na rychlosti kol
    L = read_only_vars.agent_drive.interwheel_dist;
    vR = v + (L/2)*w;
    vL = v - (L/2)*w;

    % saturace
    maxVel = read_only_vars.agent_drive.max_vel;
    vR = max(min(vR, maxVel), -maxVel);
    vL = max(min(vL, maxVel), -maxVel);

    motion_vector = [vR, vL];
end