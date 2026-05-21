function public_vars = init_kalman_filter(read_only_vars, public_vars)
% INIT_KALMAN_FILTER
% EKF for GNSS-based localization.
%
% State:
%   x = [x; y; theta]
%
% GNSS measurement:
%   z = [x; y]

    public_vars.kf.C = [1 0 0;
                        0 1 0];

    public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;

    % Q = process noise, 3x3 because state is [x y theta]
    public_vars.kf.Q = diag([0.015, 0.015, 0.005]);

    % R = GNSS measurement noise, 2x2 because GNSS gives [x y]
    % Increase this if EKF jumps too much after GNSS points.
    public_vars.kf.R = diag([0.8, 0.8]);

    % Final project: start is not known, initialize from GNSS when available.
    public_vars.use_gnss_initialization = true;

    public_vars.mu = [0; 0; 0];
    public_vars.sigma = diag([100, 100, 10]);

    public_vars.gnss_init_buffer = [];
    public_vars.gnss_init_samples_needed = 10;
    public_vars.gnss_initialized = false;

    % GNSS does not measure heading directly.
    public_vars.initial_theta_variance = 4.0;
end