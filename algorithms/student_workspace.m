function public_vars = student_workspace(read_only_vars, public_vars)
% STUDENT_WORKSPACE

    % -------------------------
    % Initialization
    % -------------------------
    if ~isfield(public_vars, 'kf_initialized')
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.kf_initialized = true;

        public_vars.motion_vector = [0, 0];
        public_vars.motion_vector_prev = [0, 0];
    end

    % -------------------------
    % Task 4: GNSS-based initialization
    % Robot stands still and collects GNSS samples
    % -------------------------
    if public_vars.use_gnss_initialization && ~public_vars.gnss_initialized

        z = read_only_vars.gnss_position(:);

        if all(isfinite(z))
            public_vars.gnss_init_buffer = [public_vars.gnss_init_buffer; z(:)'];
        end

        % Wait until enough samples are collected
        if size(public_vars.gnss_init_buffer, 1) >= public_vars.gnss_init_samples_needed

            X = public_vars.gnss_init_buffer;

            % Mean GNSS position
            mu_xy = mean(X, 1)';

            % GNSS covariance
            Q_xy = cov(X);
            % Use estimated GNSS covariance as measurement covariance
            public_vars.kf.Q = Q_xy;

            % Initial state belief
            public_vars.mu = [mu_xy(1); mu_xy(2); 0];

            % Initial covariance:
            % x,y from GNSS covariance, theta large variance
            public_vars.sigma = [Q_xy(1,1), Q_xy(1,2), 0;
                                 Q_xy(2,1), Q_xy(2,2), 0;
                                 0,         0,         public_vars.initial_theta_variance];

            public_vars.gnss_initialized = true;
        end

        % During GNSS initialization robot must stand still
        public_vars.motion_vector = [0, 0];
        public_vars.motion_vector_prev = [0, 0];

        % Estimated pose for visualization
        public_vars.estimated_pose = public_vars.mu(:)';

        return;
    end

    % -------------------------
    % Normal EKF operation
    % -------------------------

    % EKF uses previous control input
    public_vars.motion_vector = public_vars.motion_vector_prev;
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

    % Estimated pose for visualization
    public_vars.estimated_pose = public_vars.mu(:)';

    % Compute new motion control from EKF estimate
    [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars);

    % Save command for next step
    public_vars.motion_vector_prev = motion_vector;
    public_vars.motion_vector = motion_vector;
end