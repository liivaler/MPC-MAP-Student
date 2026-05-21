function [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars)
% PLAN_MOTION
%
% Functional rotate-first version:
% - uses path_idx
% - startup_align_done for first rotation
% - output convention: motion_vector = [leftWheel rightWheel]

    % ============================================================
    % 0) Basic checks
    % ============================================================
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
    theta = wrap_angle_local(pose(3));

    path = public_vars.path;
    N = size(path,1);

    if N < 2
        motion_vector = [0 0];
        return;
    end

    goal = read_only_vars.map.goal(1:2);
    goal = goal(:)';

    if isfield(read_only_vars.map, 'goal_tolerance')
        goal_tol = min(read_only_vars.map.goal_tolerance, 0.50);
    else
        goal_tol = 0.40;
    end

    if hypot(x - goal(1), y - goal(2)) < goal_tol
        motion_vector = [0 0];
        return;
    end

    public_vars.path_idx = min(max(round(public_vars.path_idx), 1), N);

    % ============================================================
    % 1) GNSS / outdoor detection
    % ============================================================
    gnss_ok = isfield(read_only_vars,'gnss_position') && ...
              numel(read_only_vars.gnss_position) >= 2 && ...
              all(isfinite(read_only_vars.gnss_position(1:2)));

    if gnss_ok
        % OUTDOOR - speed values unchanged
        wp_tol = 0.35;
        forward_window = 10;
        lookahead_steps = 2;

        v_nom = 0.5;
        w_gain = 1.15;
        w_lim = 1.05;
        maxVelLocal = 0.80;

        % Only outdoor path-following geometry changes.
        corner_threshold = deg2rad(25);
        corner_lock_dist = 0.75;
        k_cte = 0.65;
        max_cte_angle = 0.55;

    else
        % INDOOR - original behavior
        wp_tol = 0.60;
        forward_window = 6;
        lookahead_steps = 1;

        v_nom = 0.60;
        w_gain = 1.15;
        w_lim = 1.05;
        maxVelLocal = 0.65;
    end

    % ============================================================
    % 2) Path index update using forward window
    % ============================================================
    current_idx = round(min(max(public_vars.path_idx, 1), N));

    search_from = current_idx;
    search_to   = min(N, current_idx + forward_window);

    sub_path = path(search_from:search_to, :);

    d_sub = hypot(sub_path(:,1) - x, sub_path(:,2) - y);
    [~, best_local_idx] = min(d_sub);

    best_idx = search_from + best_local_idx - 1;

    if best_idx > public_vars.path_idx
        public_vars.path_idx = best_idx;
    end

    while public_vars.path_idx < N
        wp = path(public_vars.path_idx,:);
        d_wp = hypot(x - wp(1), y - wp(2));

        if d_wp < wp_tol
            public_vars.path_idx = public_vars.path_idx + 1;
        else
            break;
        end
    end

    public_vars.path_idx = min(max(round(public_vars.path_idx), 1), N);

    % ============================================================
    % 3) Direction error
    % ============================================================
    if gnss_ok
        % --------------------------------------------------------
        % OUTDOOR:
        % Follow the current trajectory segment, not a far target point.
        % This prevents cutting corners / making early arcs.
        % --------------------------------------------------------

        idx_seg = min(max(public_vars.path_idx, 1), N-1);

        % Detect upcoming corner.
        corner_idx = 0;

        scan_to = min(N-1, idx_seg + 10);

        for k = idx_seg:scan_to
            v1 = path(k+1,:) - path(k,:);

            if norm(v1) < 1e-6
                continue;
            end

            if k+1 <= N-1
                v2 = path(k+2,:) - path(k+1,:);

                if norm(v2) < 1e-6
                    continue;
                end

                a1 = atan2(v1(2), v1(1));
                a2 = atan2(v2(2), v2(1));

                da = abs(wrap_angle_local(a2 - a1));

                if da > corner_threshold
                    corner_idx = k + 1;
                    break;
                end
            end
        end

        % If a corner is ahead and robot is not close to it,
        if corner_idx > 0
            d_corner = hypot(path(corner_idx,1) - x, path(corner_idx,2) - y);

            if d_corner > corner_lock_dist
                idx_seg = max(1, min(corner_idx - 1, N-1));
            else
                idx_seg = max(1, min(corner_idx, N-1));
            end
        end

        pA = path(idx_seg, :);
        pB = path(idx_seg + 1, :);

        seg = pB - pA;
        seg_len = norm(seg);

        if seg_len < 1e-6
            target_angle = theta;
            cte_angle = 0;
        else
            tx = seg(1) / seg_len;
            ty = seg(2) / seg_len;

            target_angle = atan2(ty, tx);

            % Signed cross-track error to current segment.
            % Positive means robot is left of segment.
            rx = x - pA(1);
            ry = y - pA(2);

            cte = tx * ry - ty * rx;

            % For path to the right/east:
            % if robot is left of the segment, steer right,
            % therefore subtract cte correction.
            cte_angle = atan(k_cte * cte);
            cte_angle = max(-max_cte_angle, min(max_cte_angle, cte_angle));
        end

        heading_err = wrap_angle_local(target_angle - theta);

        % heading to segment + correction back to line.
        err = wrap_angle_local(heading_err - cte_angle);

        % For startup alignment, use only segment direction,
        % not cross-track correction.
        align_err = heading_err;

    else
        % --------------------------------------------------------
        % INDOOR:
        % Keep old target-point behavior.
        % --------------------------------------------------------
        target_idx = min(public_vars.path_idx + lookahead_steps, N);
        target = path(target_idx,:);

        dx = target(1) - x;
        dy = target(2) - y;

        if hypot(dx, dy) < 1e-6
            motion_vector = [0 0];
            return;
        end

        target_angle = atan2(dy, dx);
        err = wrap_angle_local(target_angle - theta);
        align_err = err;
    end

    abs_err = abs(err);

    % ============================================================
    % 4) Startup alignment after new path
    % ============================================================
    if ~isfield(public_vars, 'startup_align_done')
        public_vars.startup_align_done = false;
    end

    % Allow driving only after robot is facing trajectory direction.
    if ~public_vars.startup_align_done && abs(align_err) < 0.18
        public_vars.startup_align_done = true;
    end

    startup_align_active = ~public_vars.startup_align_done;

    % ============================================================
    % 5) Steering
    % ============================================================
    w = w_gain * err;
    w = max(min(w, w_lim), -w_lim);

    if abs_err < 0.04
        w = 0;
    end

    % ============================================================
    % 6) Speed
    % ============================================================
    if startup_align_active
        % Rotate only after new path.
        v = 0.00;

        w_align = w_gain * align_err;
        w_align = max(min(w_align, w_lim), -w_lim);

        if abs(w_align) < 0.28 && abs(align_err) > 0.04
            w_align = 0.28 * sign(align_err);
        end

        w = w_align;

    elseif abs_err > 1.25
        % If direction is badly wrong, rotate again.
        v = 0.00;

        if abs(w) < 0.30
            w = 0.30 * sign(err);
        end

    else
        % Normal driving.
        v = v_nom * max(0.30, cos(err));

        if abs_err > 0.85
            v = min(v, 0.20);
        elseif abs_err > 0.55
            v = min(v, 0.35);
        elseif abs_err > 0.30
            v = min(v, 0.55);
        end
    end

    % Slow down near goal.
    d_goal = hypot(x - goal(1), y - goal(2));

    if d_goal < 0.75
        v = min(v, 0.25);
    end

    % ============================================================
    % 7) Differential drive conversion
    % ============================================================
    L = read_only_vars.agent_drive.interwheel_dist;

    vR = v - (L/2)*w;
    vL = v + (L/2)*w;

    if isfield(read_only_vars,'agent_drive') && ...
       isfield(read_only_vars.agent_drive,'max_vel')
        maxVel = min(maxVelLocal, read_only_vars.agent_drive.max_vel);
    else
        maxVel = maxVelLocal;
    end

    vR = max(min(vR, maxVel), -maxVel);
    vL = max(min(vL, maxVel), -maxVel);

    motion_vector = [vL vR];

    % ============================================================
    % 8) Sanity fallback
    % ============================================================
    if numel(motion_vector) ~= 2 || any(~isfinite(motion_vector))
        motion_vector = [0 0];
    end
end


function a = wrap_angle_local(a)
    a = atan2(sin(a), cos(a));
end