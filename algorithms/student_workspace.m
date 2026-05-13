function public_vars = student_workspace(read_only_vars, public_vars)
% STUDENT_WORKSPACE
% Final project workspace.
%
% No MoCap is used.
% Outdoor: EKF/GNSS.
% Indoor / GNSS denied: PF + lidar.

    % ============================================================
    % 0) Initialization
    % ============================================================
    if ~isfield(public_vars, 'initialized')
        public_vars.initialized = true;

        public_vars.motion_vector = [0 0];
        public_vars.prev_motion_vector = [0 0];

        public_vars.path = [];
        public_vars.path_idx = 1;
        public_vars.replan_counter = 999;

        public_vars.pf_lock_counter = 0;
        public_vars.pf_wait_counter = 0;
        public_vars.pf_global_search = true;
        public_vars.pf_seeded_from_gnss = false;

        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars = init_particle_filter(read_only_vars, public_vars);

        if ~isfield(public_vars, 'gnss_initialized')
            public_vars.gnss_initialized = false;
        end

        public_vars.last_gnss_ok = false;

        % Important: never start with fake pose [0 0 0].
        public_vars.estimated_pose = [NaN NaN NaN];
        public_vars.estimated_pose_filt = [NaN NaN NaN];
        public_vars.pose_valid = false;
        public_vars.last_mode_id = 0;

        public_vars.stuck_counter = 0;
        public_vars.last_pose_check = [NaN NaN NaN];

        public_vars.localization_fail_counter = 0;
        public_vars.last_goal_distance = inf;
        public_vars.relocalization_counter = 0;

        public_vars.transition_counter = 999;

        public_vars.emergency_mode = false;
        public_vars.emergency_counter = 0;
        public_vars.search_counter = 0;

        % ========================================================
        % Internal counters
        % ========================================================
        public_vars.loop_counter = 0;
        public_vars.replan_total = 0;
        public_vars.pf_search_total = 0;
        public_vars.emergency_total = 0;
        public_vars.stuck_total = 0;
        public_vars.goal_reached_loop = NaN;
    end

    % ============================================================
    % Loop counter
    % ============================================================
    public_vars.loop_counter = public_vars.loop_counter + 1;

    % ============================================================
    % 1) GNSS availability
    % ============================================================
    z = read_only_vars.gnss_position(:);
    gnss_ok = numel(z) >= 2 && all(isfinite(z(1:2)));

    % ============================================================
    % 2) EKF always runs
    % ============================================================
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

    if gnss_ok
        public_vars.gnss_initialized = true;
        public_vars.pose_valid = true;

        public_vars.pf_global_search = false;
        public_vars.pf_seeded_from_gnss = false;
        public_vars.pf_wait_counter = 0;
        public_vars.transition_counter = 999;
    end

    % ============================================================
    % 3) Outdoor -> indoor / GNSS denied transition
    % ============================================================
    if public_vars.last_gnss_ok && ~gnss_ok

        public_vars.transition_counter = 0;

        % Seed particles around last EKF/GNSS estimate.
        public_vars = seed_particles_around_pose( ...
            public_vars, ...
            public_vars.mu(:)', ...
            read_only_vars);

        public_vars.pf_lock_counter = 0;
        public_vars.pf_wait_counter = 0;
        public_vars.pf_global_search = false;
        public_vars.pf_seeded_from_gnss = true;

        public_vars.path = [];
        public_vars.path_idx = 1;
        public_vars.replan_counter = 999;

        public_vars.estimated_pose = public_vars.mu(:)';
        public_vars.estimated_pose(3) = wrap_angle(public_vars.estimated_pose(3));
        public_vars.estimated_pose_filt = public_vars.estimated_pose;

        public_vars.pose_valid = true;
        public_vars.last_mode_id = 2;

        public_vars.stuck_counter = 0;
        public_vars.last_pose_check = public_vars.estimated_pose;

        public_vars.localization_fail_counter = 0;
        public_vars.last_goal_distance = inf;

        public_vars.emergency_mode = false;
        public_vars.emergency_counter = 0;
        public_vars.search_counter = 0;
    end

    public_vars.last_gnss_ok = gnss_ok;

    if ~gnss_ok
        public_vars.transition_counter = public_vars.transition_counter + 1;
    end

    % ============================================================
    % 4) PF when GNSS is not available
    % ============================================================
    if ~gnss_ok

        [public_vars.particles, public_vars.weights] = ...
            update_particle_filter(read_only_vars, public_vars);

        pf_pose = estimate_pose(public_vars);
        pf_pose(3) = wrap_angle(pf_pose(3));

        spread = particle_spread(public_vars.particles, public_vars.weights);
        cluster_mass = particle_cluster_mass(public_vars.particles, public_vars.weights);

        % --------------------------------------------------------
        % PF lock conditions
        % --------------------------------------------------------
        if public_vars.pf_seeded_from_gnss
            required_lock_count = 10;
            spread_limit = 0.80;
            mass_limit = 0.08;
            wait_before_planning = 11;
        else
            required_lock_count = 18;
            spread_limit = 0.70;
            mass_limit = 0.06;
            wait_before_planning = 14;
        end

        % Lock according to particle density, not according to scan_error.
        if spread < spread_limit && cluster_mass > mass_limit
            public_vars.pf_lock_counter = public_vars.pf_lock_counter + 1;

        elseif spread < 1.30 && cluster_mass > 0.04
            public_vars.pf_lock_counter = max(public_vars.pf_lock_counter - 1, 0);

        else
            public_vars.pf_lock_counter = 0;
        end

        % Do not validate too early.
        if public_vars.pf_lock_counter >= 5
            public_vars.pose_valid = true;
        else
            if ~public_vars.pf_seeded_from_gnss
                public_vars.pose_valid = false;
            end
        end

        % --------------------------------------------------------
        % Short localization-only phase
        % --------------------------------------------------------
        if public_vars.pf_lock_counter < required_lock_count

            public_vars.pf_search_total = public_vars.pf_search_total + 1;
            public_vars.pf_wait_counter = public_vars.pf_wait_counter + 1;

            public_vars.estimated_pose = pf_pose;
            public_vars.estimated_pose(3) = wrap_angle(public_vars.estimated_pose(3));

            if public_vars.pf_wait_counter < wait_before_planning

                public_vars.path = [];
                public_vars.path_idx = 1;
                public_vars.replan_counter = 999;

                % Small stable arc to collect different lidar views.
                public_vars.motion_vector = [0.07 -0.07];
                public_vars.motion_vector = safety_lidar( ...
                    public_vars.motion_vector, ...
                    read_only_vars, ...
                    public_vars.estimated_pose);

                public_vars.prev_motion_vector = public_vars.motion_vector;
                return;
            end
        end
    end

    % ============================================================
    % 5) Choose localization source
    % ============================================================
    if gnss_ok && public_vars.gnss_initialized
        public_vars.estimated_pose = public_vars.mu(:)';
        public_vars.estimated_pose(3) = wrap_angle(public_vars.estimated_pose(3));
        mode_id = 1;
        public_vars.pose_valid = true;

    else
        public_vars.estimated_pose = estimate_pose(public_vars);
        public_vars.estimated_pose(3) = wrap_angle(public_vars.estimated_pose(3));
        mode_id = 2;
    end

    if numel(public_vars.estimated_pose) ~= 3 || any(~isfinite(public_vars.estimated_pose))

        public_vars.pose_valid = false;

        public_vars.path = [];
        public_vars.path_idx = 1;
        public_vars.replan_counter = 999;

        public_vars.motion_vector = [0.08 0.05];
        public_vars.motion_vector = safety_lidar(public_vars.motion_vector, read_only_vars);
        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    % ============================================================
    % 6) Pose filtering
    % ============================================================
    if ~isfield(public_vars, 'estimated_pose_filt') || ...
       numel(public_vars.estimated_pose_filt) ~= 3 || ...
       any(~isfinite(public_vars.estimated_pose_filt)) || ...
       ~isfield(public_vars, 'pose_valid') || ~public_vars.pose_valid

        public_vars.estimated_pose_filt = public_vars.estimated_pose;
        public_vars.last_mode_id = mode_id;

    elseif ~isfield(public_vars, 'last_mode_id') || public_vars.last_mode_id ~= mode_id

        public_vars.estimated_pose_filt = public_vars.estimated_pose;
        public_vars.last_mode_id = mode_id;

        public_vars.stuck_counter = 0;
        public_vars.last_pose_check = public_vars.estimated_pose;

    else
        if mode_id == 1
            alpha = 0.25;     % GNSS/EKF
        else
            alpha = 0.10;     % PF smoother, less jumpy indoors
        end

        old = public_vars.estimated_pose_filt;
        new = public_vars.estimated_pose;

        public_vars.estimated_pose_filt(1:2) = ...
            (1-alpha)*old(1:2) + alpha*new(1:2);

        dtheta = wrap_angle(new(3) - old(3));
        public_vars.estimated_pose_filt(3) = wrap_angle(old(3) + alpha*dtheta);
    end

    public_vars.estimated_pose = public_vars.estimated_pose_filt;
    public_vars.estimated_pose(3) = wrap_angle(public_vars.estimated_pose(3));

    % ============================================================
    % 7) Path planning
    % ============================================================
    public_vars.replan_counter = public_vars.replan_counter + 1;

    need_replan = isempty(public_vars.path);

    if ~isempty(public_vars.path)

        d_path = distance_to_path(public_vars.estimated_pose(1:2), public_vars.path);

        if d_path > 0.6 && d_path <= 2.0
            public_vars.path_idx = nearest_path_index( ...
                public_vars.estimated_pose(1:2), ...
                public_vars.path);
        end

        if public_vars.replan_counter > 160 && d_path > 2.0
            need_replan = true;
        end
    end

    pf_can_plan = false;
    pf_can_follow = false;

    if gnss_ok
        pf_can_plan = true;
        pf_can_follow = true;
    else
        if public_vars.pf_wait_counter >= required_pf_wait(public_vars)
            pf_can_plan = true;
        end

        % More strict: avoid following false symmetric cluster.
        if public_vars.pf_lock_counter >= 10
            pf_can_follow = true;
        end
    end

    % In symmetric indoor maps do not plan from an unvalidated PF pose.
    can_plan = gnss_ok || ...
               (pf_can_plan && public_vars.pose_valid);

    can_follow_path = gnss_ok || ...
                      (pf_can_follow && public_vars.pose_valid);

    if need_replan && can_plan && public_vars.replan_counter > 25
        public_vars.path = plan_path(read_only_vars, public_vars);
        public_vars.path_idx = 1;
        public_vars.replan_counter = 0;

        public_vars.replan_total = public_vars.replan_total + 1;
    end

    % ============================================================
    % 8) Stuck detection
    % ============================================================
    if ~isfield(public_vars,'stuck_counter')
        public_vars.stuck_counter = 0;
    end

    if ~isfield(public_vars,'last_pose_check') || ...
       numel(public_vars.last_pose_check) ~= 3 || ...
       any(~isfinite(public_vars.last_pose_check))

        public_vars.last_pose_check = public_vars.estimated_pose;
    end

    move_dist = hypot( ...
        public_vars.estimated_pose(1) - public_vars.last_pose_check(1), ...
        public_vars.estimated_pose(2) - public_vars.last_pose_check(2));

    if move_dist < 0.05
        public_vars.stuck_counter = public_vars.stuck_counter + 1;
    else
        public_vars.stuck_counter = 0;
        public_vars.last_pose_check = public_vars.estimated_pose;
    end

    % ============================================================
    % 9) Localization watchdog
    % ============================================================
    goal = read_only_vars.map.goal(1:2);

    goal_dist = hypot( ...
        public_vars.estimated_pose(1) - goal(1), ...
        public_vars.estimated_pose(2) - goal(2));

    if ~isfield(public_vars, 'last_goal_distance') || ...
       ~isfinite(public_vars.last_goal_distance)
        public_vars.last_goal_distance = goal_dist;
    end

    cmd_mag = max(abs(public_vars.prev_motion_vector));
    progress = public_vars.last_goal_distance - goal_dist;

    if cmd_mag > 0.03 && progress < 0.005 && ~gnss_ok
        public_vars.localization_fail_counter = public_vars.localization_fail_counter + 1;
    else
        public_vars.localization_fail_counter = max(public_vars.localization_fail_counter - 2, 0);
    end

    public_vars.last_goal_distance = goal_dist;

    if public_vars.localization_fail_counter > 160
        public_vars = reset_localization(read_only_vars, public_vars);

        public_vars.motion_vector = [0.08 0.05];
        public_vars.motion_vector = safety_lidar( ...
            public_vars.motion_vector, ...
            read_only_vars, ...
            public_vars.estimated_pose);

        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    % ============================================================
    % 10) Motion
    % ============================================================

    % Emergency obstacle handling must be before all motion returns.
    [emergency, ~] = obstacle_emergency(read_only_vars);

    if emergency || public_vars.emergency_mode

        public_vars.emergency_total = public_vars.emergency_total + 1;
    
        public_vars.emergency_mode = true;
        public_vars.emergency_counter = public_vars.emergency_counter + 1;
    
        if public_vars.emergency_counter == 1
            public_vars.path = [];
            public_vars.path_idx = 1;
            public_vars.replan_counter = 999;
        end
    
        % 1) Nejdřív couvej výrazněji a déle.
        if public_vars.emergency_counter < 14
            public_vars.motion_vector = [-0.18 -0.16];
    
        % 2) Potom se otoč na místě, aby lidar našel volnější směr.
        elseif public_vars.emergency_counter < 28
            public_vars.motion_vector = [0.14 -0.14];
    
        % 3) Krátký pomalý rozjezd dopředu, ale bez agresivního plánování.
        elseif public_vars.emergency_counter < 36
            public_vars.motion_vector = [0.08 0.06];
    
        else
            public_vars.emergency_mode = false;
            public_vars.emergency_counter = 0;
    
            public_vars.path = [];
            public_vars.path_idx = 1;
            public_vars.replan_counter = 999;
    
            public_vars.motion_vector = [0.06 0.05];
        end
    
        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    if isempty(public_vars.path)

        % No trusted path yet: move slowly and collect more lidar.
        public_vars.motion_vector = [0.08 0.05];
        public_vars.motion_vector = safety_lidar( ...
            public_vars.motion_vector, ...
            read_only_vars, ...
            public_vars.estimated_pose);

        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    if public_vars.stuck_counter > 80

        public_vars.stuck_total = public_vars.stuck_total + 1;

        public_vars = reset_localization(read_only_vars, public_vars);

        public_vars.motion_vector = [-0.12 0.12];
        public_vars.prev_motion_vector = public_vars.motion_vector;

        public_vars.stuck_counter = 0;

        return;
    end

    if ~gnss_ok && ~can_follow_path

        if ~isfield(public_vars, 'search_counter')
            public_vars.search_counter = 0;
        end

        public_vars.search_counter = public_vars.search_counter + 1;

        if public_vars.search_counter < 25
            public_vars.motion_vector = [0.08 0.05];

        elseif public_vars.search_counter < 55
            public_vars.motion_vector = [0.06 -0.06];

        else
            public_vars.search_counter = 0;

            public_vars.path = [];
            public_vars.path_idx = 1;
            public_vars.replan_counter = 999;

            public_vars.motion_vector = [0.08 0.05];
        end

        public_vars.motion_vector = safety_lidar( ...
            public_vars.motion_vector, ...
            read_only_vars, ...
            public_vars.estimated_pose);

        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    [mv, public_vars] = plan_motion(read_only_vars, public_vars);
    public_vars.motion_vector = mv(:)';

    if numel(public_vars.motion_vector) ~= 2 || any(~isfinite(public_vars.motion_vector))
        public_vars.motion_vector = [0 0];
    end

    goal_tol = min(read_only_vars.map.goal_tolerance, 0.50);

    d_goal = hypot( ...
        public_vars.estimated_pose(1) - goal(1), ...
        public_vars.estimated_pose(2) - goal(2));
    
    goal_stop_tol = min(0.25, 0.60 * goal_tol);

    if d_goal < goal_stop_tol

        if ~isfield(public_vars, 'goal_reached_loop') || isnan(public_vars.goal_reached_loop)
            public_vars.goal_reached_loop = public_vars.loop_counter;
        end

        public_vars.motion_vector = [0 0];
        public_vars.prev_motion_vector = public_vars.motion_vector;
        return;
    end

    public_vars.motion_vector = safety_lidar( ...
        public_vars.motion_vector, ...
        read_only_vars, ...
        public_vars.estimated_pose);

    public_vars.prev_motion_vector = public_vars.motion_vector;
end


% ========================================================================
% Helper functions
% ========================================================================

function public_vars = reset_localization(read_only_vars, public_vars)

    public_vars.path = [];
    public_vars.path_idx = 1;
    public_vars.replan_counter = 999;

    public_vars.pf_lock_counter = 0;
    public_vars.pf_wait_counter = 0;
    public_vars.localization_fail_counter = 0;

    if ~isfield(public_vars, 'relocalization_counter')
        public_vars.relocalization_counter = 0;
    end

    public_vars.relocalization_counter = public_vars.relocalization_counter + 1;

    public_vars.pf_global_search = true;
    public_vars.pf_seeded_from_gnss = false;
    public_vars.pose_valid = false;

    public_vars.emergency_mode = false;
    public_vars.emergency_counter = 0;
    public_vars.search_counter = 0;

    public_vars = init_particle_filter(read_only_vars, public_vars);
end

function n = required_pf_wait(public_vars)

    if isfield(public_vars, 'pf_seeded_from_gnss') && public_vars.pf_seeded_from_gnss
        n = 8;
    else
        n = 10;
    end
end


function a = wrap_angle(a)

    a = atan2(sin(a), cos(a));
end


function dmin = distance_to_path(p, path)

    p = p(:)';

    if isempty(path)
        dmin = inf;
        return;
    end

    d = hypot(path(:,1) - p(1), path(:,2) - p(2));
    dmin = min(d);
end


function idx = nearest_path_index(p, path)

    p = p(:)';

    d = hypot(path(:,1) - p(1), path(:,2) - p(2));
    [~, idx] = min(d);

    idx = max(1, min(idx, size(path,1)));
end


function motion_vector = safety_lidar(motion_vector, read_only_vars, pose)
% SAFETY_LIDAR
% Soft safety: slows down before obstacles.
% Emergency reverse is handled separately by obstacle_emergency().

    if numel(motion_vector) ~= 2 || any(~isfinite(motion_vector))
        motion_vector = [0 0];
        return;
    end

    if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
        return;
    end

    if ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
        return;
    end

    z = read_only_vars.lidar_distances(:);
    a = read_only_vars.lidar_config(:);

    valid = isfinite(z) & z > 0;
    z = z(valid);
    a = a(valid);

    if isempty(z)
        return;
    end

    a = wrap_angle(a);

    v_forward = 0.5 * (motion_vector(1) + motion_vector(2));

    if v_forward <= 0
        return;
    end

    front_wide = abs(a) < pi/3;

    if any(front_wide)
        min_front_wide = min(z(front_wide));

        if min_front_wide < 0.2
            motion_vector = 0.4 * motion_vector;
        elseif min_front_wide < 0.3
            motion_vector = 0.70 * motion_vector;
        end
    end

    % Optional map-based predicted collision check.
    if nargin < 3 || numel(pose) ~= 3 || any(~isfinite(pose))
        return;
    end

    if ~isfield(read_only_vars, 'map') || ...
       ~isfield(read_only_vars.map, 'walls') || ...
       isempty(read_only_vars.map.walls)
        return;
    end

    L = read_only_vars.agent_drive.interwheel_dist;

    vR = motion_vector(1);
    vL = motion_vector(2);

    v = 0.5 * (vR + vL);
    w = (vR - vL) / L;

    x = pose(1);
    y = pose(2);
    theta = wrap_angle(pose(3));

    lookahead_time = 0.45;

    x_pred = x + v * cos(theta) * lookahead_time;
    y_pred = y + v * sin(theta) * lookahead_time;
    th_pred = theta + w * lookahead_time;

    nose_dist = 0.25;

    x_nose = x_pred + nose_dist * cos(th_pred);
    y_nose = y_pred + nose_dist * sin(th_pred);

    clearance = 0.18;

    if ~point_safe_from_walls(x_pred, y_pred, read_only_vars.map.walls, clearance) || ...
       ~point_safe_from_walls(x_nose, y_nose, read_only_vars.map.walls, clearance)

        motion_vector = 0.45 * motion_vector;
    end
end


function [emergency, min_front] = obstacle_emergency(read_only_vars)

    emergency = false;
    min_front = inf;

    if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
        return;
    end

    if ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
        return;
    end

    z = read_only_vars.lidar_distances(:);
    a = read_only_vars.lidar_config(:);

    valid = isfinite(z) & z > 0;
    z = z(valid);
    a = a(valid);

    if isempty(z)
        return;
    end

    a = wrap_angle(a);

    front = abs(a) < pi/4;

    if ~any(front)
        return;
    end

    min_front = min(z(front));

    if min_front < 0.12
        emergency = true;
    end
end


function e = lidar_pose_error(read_only_vars, pose)

    e = inf;

    if numel(pose) ~= 3 || any(~isfinite(pose))
        return;
    end

    if ~isfield(read_only_vars,'lidar_distances') || isempty(read_only_vars.lidar_distances)
        return;
    end

    if ~isfield(read_only_vars,'lidar_config') || isempty(read_only_vars.lidar_config)
        return;
    end

    z_real = read_only_vars.lidar_distances(:)';

    z_pred = compute_lidar_measurement( ...
        read_only_vars.map, ...
        pose, ...
        read_only_vars.lidar_config);

    M = min(numel(z_real), numel(z_pred));

    z_real = z_real(1:M);
    z_pred = z_pred(1:M);

    valid = isfinite(z_real) & isfinite(z_pred) & z_real > 0 & z_pred > 0;

    if sum(valid) < 3
        return;
    end

    err = z_real(valid) - z_pred(valid);
    err = max(-2.0, min(2.0, err));

    e = sqrt(mean(err.^2));
end


function spread = particle_spread(particles, weights)
% PARTICLE_SPREAD
% Density-based spread, compatible with resampled PF where weights may be uniform.

    N = size(particles,1);

    if N == 0
        spread = inf;
        return;
    end

    cluster_radius = 0.75;

    Kcand = min(120, N);
    candidates = unique(round(linspace(1, N, Kcand)));

    best_count = 0;
    best_center_idx = candidates(1);

    for k = 1:numel(candidates)

        idx = candidates(k);

        cx = particles(idx,1);
        cy = particles(idx,2);

        d = hypot(particles(:,1) - cx, particles(:,2) - cy);
        local = d < cluster_radius;

        cnt = sum(local);

        if cnt > best_count
            best_count = cnt;
            best_center_idx = idx;
        end
    end

    cx = particles(best_center_idx,1);
    cy = particles(best_center_idx,2);

    d = hypot(particles(:,1) - cx, particles(:,2) - cy);
    local = d < cluster_radius;

    if sum(local) < 15
        spread = inf;
        return;
    end

    spread = sqrt(mean(d(local).^2));
end


function mass = particle_cluster_mass(particles, weights)
% PARTICLE_CLUSTER_MASS
% Density-based cluster mass = fraction of particles in strongest cluster.

    N = size(particles,1);

    if N == 0
        mass = 0;
        return;
    end

    cluster_radius = 0.75;

    Kcand = min(120, N);
    candidates = unique(round(linspace(1, N, Kcand)));

    best_count = 0;

    for k = 1:numel(candidates)

        idx = candidates(k);

        cx = particles(idx,1);
        cy = particles(idx,2);

        d = hypot(particles(:,1) - cx, particles(:,2) - cy);
        local = d < cluster_radius;

        cnt = sum(local);

        if cnt > best_count
            best_count = cnt;
        end
    end

    mass = best_count / N;
end


function public_vars = seed_particles_around_pose(public_vars, pose, read_only_vars)

    N = size(public_vars.particles,1);

    if N == 0
        return;
    end

    pose = pose(:)';

    for i = 1:N

        ok = false;

        for k = 1:200

            p = pose;

            p(1) = pose(1) + 0.45*randn;
            p(2) = pose(2) + 0.45*randn;
            p(3) = wrap_angle(pose(3) + 0.60*randn);

            if is_pose_safe_local(p, read_only_vars, 0.12)
                public_vars.particles(i,:) = p;
                ok = true;
                break;
            end
        end

        if ~ok
            public_vars.particles(i,:) = pose;
            public_vars.particles(i,3) = wrap_angle(public_vars.particles(i,3));
        end
    end

    public_vars.weights = ones(N,1) / N;
end


function ok = is_pose_safe_local(p, read_only_vars, clearance)

    x = p(1);
    y = p(2);

    limits = read_only_vars.map.limits;

    ok = x > limits(1) && x < limits(3) && ...
         y > limits(2) && y < limits(4);

    if ~ok
        return;
    end

    if ~isfield(read_only_vars.map,'walls') || isempty(read_only_vars.map.walls)
        return;
    end

    ok = point_safe_from_walls(x, y, read_only_vars.map.walls, clearance);
end


function ok = point_safe_from_walls(x, y, walls, clearance)

    ok = true;

    for j = 1:size(walls,1)

        d = point_to_segment_distance( ...
            x, y, ...
            walls(j,1), walls(j,2), ...
            walls(j,3), walls(j,4));

        if d < clearance
            ok = false;
            return;
        end
    end
end


function d = point_to_segment_distance(px, py, x1, y1, x2, y2)

    vx = x2 - x1;
    vy = y2 - y1;

    wx = px - x1;
    wy = py - y1;

    len2 = vx*vx + vy*vy;

    if len2 == 0
        d = hypot(px - x1, py - y1);
        return;
    end

    t = (wx*vx + wy*vy) / len2;
    t = max(0, min(1, t));

    proj_x = x1 + t*vx;
    proj_y = y1 + t*vy;

    d = hypot(px - proj_x, py - proj_y);
end