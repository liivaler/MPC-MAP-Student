function public_vars = student_workspace(read_only_vars, public_vars)
% STUDENT_WORKSPACE - Week 6

    if ~isfield(public_vars, 'initialized')

        public_vars.initialized = true;

        % Initial pose
        if isfield(read_only_vars, 'mocap_pose') && all(isfinite(read_only_vars.mocap_pose(1:3)))
            public_vars.estimated_pose = read_only_vars.mocap_pose;
        else
            public_vars.estimated_pose = [1, 1, pi/2];
        end

        % Plan path once
        public_vars.path = plan_path(read_only_vars, public_vars);

        % Make first path point exactly equal to robot start position
        if ~isempty(public_vars.path)
            public_vars.path(1,:) = public_vars.estimated_pose(1:2);
        end

        if ~isempty(public_vars.path) && size(public_vars.path,1) >= 2
            public_vars.wp_idx = 2;
        else
            public_vars.wp_idx = 1;
        end

        public_vars.motion_vector = [0, 0];
        public_vars.prev_motion_vector = [0, 0];

        disp('Planned path size:');
        disp(size(public_vars.path));
    end

    % ------------------------------------------------------------
    % Current pose
    % ------------------------------------------------------------
    if isfield(read_only_vars, 'mocap_pose') && all(isfinite(read_only_vars.mocap_pose(1:3)))

        pose = read_only_vars.mocap_pose;

    else

        pose = public_vars.estimated_pose;

        vR_old = public_vars.prev_motion_vector(1);
        vL_old = public_vars.prev_motion_vector(2);

        L = read_only_vars.agent_drive.interwheel_dist;
        dt = read_only_vars.sampling_period;

        v_old = (vR_old + vL_old) / 2;
        w_old = (vR_old - vL_old) / L;

        pose(1) = pose(1) + v_old * cos(pose(3)) * dt;
        pose(2) = pose(2) + v_old * sin(pose(3)) * dt;
        pose(3) = pose(3) + w_old * dt;
        pose(3) = atan2(sin(pose(3)), cos(pose(3)));
    end

    public_vars.estimated_pose = pose;

    % ------------------------------------------------------------
    % Stop if no path
    % ------------------------------------------------------------
    if isempty(public_vars.path)
        public_vars.motion_vector = [0, 0];
        public_vars.prev_motion_vector = [0, 0];
        return;
    end
    
    % ------------------------------------------------------------
    % Waypoint following - original simple version
    % ------------------------------------------------------------
    target = public_vars.path(public_vars.wp_idx,:);

    while norm(target - pose(1:2)) < 0.25 && public_vars.wp_idx < size(public_vars.path,1)
        public_vars.wp_idx = public_vars.wp_idx + 1;
        target = public_vars.path(public_vars.wp_idx,:);
    end

    % ------------------------------------------------------------
    % Stop at final goal
    % ------------------------------------------------------------
   if public_vars.wp_idx >= size(public_vars.path,1)
        if norm(target - pose(1:2)) < 0.2
            public_vars.motion_vector = [0, 0];
            public_vars.prev_motion_vector = [0, 0];
            return;
        end
    end
    % ------------------------------------------------------------
    % Controller - slightly improved version
    % ------------------------------------------------------------
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);

    target_heading = atan2(dy, dx);
    heading_error = atan2(sin(target_heading - pose(3)), cos(target_heading - pose(3)));

    L = read_only_vars.agent_drive.interwheel_dist;

    % Do not stop completely, only slow down when angle is large
    if abs(heading_error) > pi/3
        v = 0.1;
    else
        v = 0.25;
    end

    w = 1.5 * heading_error;
    w = max(min(w, 0.45), -0.45);

    vR = v + (L/2) * w;
    vL = v - (L/2) * w;

    maxVel = read_only_vars.agent_drive.max_vel;

    vR = max(min(vR, maxVel), -maxVel);
    vL = max(min(vL, maxVel), -maxVel);

    public_vars.motion_vector = [vR, vL];
    public_vars.prev_motion_vector = public_vars.motion_vector;
end