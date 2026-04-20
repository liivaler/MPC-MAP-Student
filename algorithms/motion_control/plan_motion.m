function [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars)
% PLAN_MOTION
% Stable path following using EKF estimate only

    if ~isfield(public_vars, 'initialized')
        public_vars.initialized = true;

        % safer smooth-ish path
        p = [ ...
            2.0, 2.0;
            2.0, 4.8;
            3.2, 6.0;
            5.0, 7.0;
            7.5, 7.2;
            10.0, 6.9;
            12.7, 6.7;
            14.0, 4.5;
            16.0, 2.0
        ];

        path = [];
        points_per_segment = 15;

        for i = 1:size(p,1)-1
            xs = linspace(p(i,1), p(i+1,1), points_per_segment)';
            ys = linspace(p(i,2), p(i+1,2), points_per_segment)';
            seg = [xs ys];

            if i < size(p,1)-1
                seg(end,:) = [];
            end

            path = [path; seg];
        end

        public_vars.path = path;

        % filtered pose memory
        public_vars.pose_filt = public_vars.mu(:)';

        % filtered wheel command memory
        public_vars.motion_filt = [0, 0];
    end

    path = public_vars.path;
    N = size(path,1);

    %% 1) EKF pose estimate
    pose = public_vars.mu(:)';
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    %% 2) filter whole pose
    alpha_p = 0.15;   % smaller = smoother
    public_vars.pose_filt = (1-alpha_p)*public_vars.pose_filt + alpha_p*[x y theta];

    x = public_vars.pose_filt(1);
    y = public_vars.pose_filt(2);
    theta = public_vars.pose_filt(3);
    theta = atan2(sin(theta), cos(theta));

    %% 3) stop near final goal
    goal = path(end,:);
    if norm([x - goal(1), y - goal(2)]) < 0.35
        motion_vector = [0, 0];
        return;
    end

    %% 4) find nearest path point
    d2 = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    [~, nearest_idx] = min(d2);

    %% 5) choose a small lookahead target
    lookahead = 4;
    target_idx = min(nearest_idx + lookahead, N);
    target = path(target_idx,:);

    dx = target(1) - x;
    dy = target(2) - y;

    target_angle = atan2(dy, dx);
    err = wrapToPi(target_angle - theta);

    %% 6) gentle controller
    v = 0.10 + 0.10*max(0, cos(err));
    w = 0.2 * err;

    % stronger slowdown in larger turns
    if abs(err) > 0.9
        v = 0.03;
    elseif abs(err) > 0.5
        v = min(v, 0.07);
    end

    % angular saturation
    w = max(min(w, 0.25), -0.25);

    %% 7) differential drive conversion
    L = read_only_vars.agent_drive.interwheel_dist;
    vR = v + (L/2)*w;
    vL = v - (L/2)*w;

    %% 8) wheel speed saturation
    maxVel = 0.35 * read_only_vars.agent_drive.max_vel;
    vR = max(min(vR, maxVel), -maxVel);
    vL = max(min(vL, maxVel), -maxVel);

    %% 9) filter output command to avoid twitching
    beta_u = 0.85;  % larger = smoother
    cmd = [vR, vL];   % simulator likely expects [left, right]
    public_vars.motion_filt = beta_u*public_vars.motion_filt + (1-beta_u)*cmd;

    motion_vector = public_vars.motion_filt;
end