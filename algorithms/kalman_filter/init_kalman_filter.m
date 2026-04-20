function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
% INIT_KALMAN_FILTER

    public_vars.kf.C = [1 0 0;
                        0 1 0];

    public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;

    % Process noise covariance
    public_vars.kf.R = diag([0.02, 0.02, 0.009]);

    % Default measurement covariance
    % For Task 4 this will be replaced by covariance estimated from GNSS
    public_vars.kf.Q = diag([0.15, 0.15]);

    
    % -------------------------------
    %Choose mode:
    % false -> Task 3 (known initial pose)
    % true  -> Task 4 (GNSS-based initialization)
    % -------------------------------
    public_vars.use_gnss_initialization = false;

    if public_vars.use_gnss_initialization
        % Temporary initial belief before GNSS initialization is completed
        public_vars.mu = [0; 0; 0];
        public_vars.sigma = diag([100, 100, 10]);

        % GNSS init buffer
        public_vars.gnss_init_buffer = [];
        public_vars.gnss_init_samples_needed = 80;
        public_vars.gnss_initialized = false;

        % Large uncertainty for heading
        public_vars.initial_theta_variance = 2.0;
    else
        % Task 3: known initial pose
        public_vars.mu = [2; 2; pi/2];
        public_vars.sigma = zeros(3,3);
        public_vars.gnss_initialized = true;
    end
end