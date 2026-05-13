function [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars)
% PLAN_MOTION
% Target-point path following with limited forward speed when heading error is large.

    if ~isfield(public_vars, 'path') || isempty(public_vars.path)
        motion_vector = [0.10 0.10];
        return;
    end

    if ~isfield(public_vars, 'path_idx')
        public_vars.path_idx = 1;
    end

    pose = public_vars.estimated_pose(:)';

    if numel(pose) ~= 3 || any(~isfinite(pose))
        motion_vector = [0 0];
        return;
    end

    x = pose(1);
    y = pose(2);
    theta = atan2(sin(pose(3)), cos(pose(3)));

    path = public_vars.path;
    N = size(path,1);

    goal = read_only_vars.map.goal(1:2);
    goal_tol = min(read_only_vars.map.goal_tolerance, 0.50);

    if hypot(x - goal(1), y - goal(2)) < goal_tol
        motion_vector = [0 0];
        return;
    end

    public_vars.path_idx = min(max(public_vars.path_idx, 1), N);

    while public_vars.path_idx < N
        wp = path(public_vars.path_idx,:);

        if hypot(x - wp(1), y - wp(2)) < 0.35
            public_vars.path_idx = public_vars.path_idx + 1;
        else
            break;
        end
    end

    % Slight lookahead smooths the path, but keep it short in indoor corridors.
    lookahead =2;
    target_idx = min(public_vars.path_idx + lookahead, N);
    target = path(target_idx,:);

    dx = target(1) - x;
    dy = target(2) - y;

    target_angle = atan2(dy, dx);
    err = atan2(sin(target_angle - theta), cos(target_angle - theta));

    % Forward speed. Do not push hard when the robot is not facing the path.
    v = 0.32 * max(0.30, cos(err));

    if abs(err) > 1.0
        v = 0.08;
    elseif abs(err) > 0.6
        v = min(v, 0.15);
    end

    % Slow down a bit near the goal.
    d_goal = hypot(x - goal(1), y - goal(2));
    if d_goal < 1.2
        v = min(v, 0.18);
    end

    w = 1.2 * err;
    w = max(min(w, 1.2), -1.2);

    L = read_only_vars.agent_drive.interwheel_dist;

    vR = v + (L/2)*w;
    vL = v - (L/2)*w;

    maxVel = 0.42;

    vR = max(min(vR, maxVel), -maxVel);
    vL = max(min(vL, maxVel), -maxVel);

    motion_vector = [vR vL];

    if all(abs(motion_vector) < 1e-4)
        motion_vector = [0.08 0.08];
    end
end
