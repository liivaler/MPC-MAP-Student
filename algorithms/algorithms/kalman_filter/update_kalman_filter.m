function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
% UPDATE_KALMAN_FILTER
% EKF update:
% 1) initialize from GNSS if not initialized
% 2) predict from wheel velocities
% 3) correct from GNSS if available

    mu = public_vars.mu;
    sigma = public_vars.sigma;

    z = read_only_vars.gnss_position(:);
    gnss_ok = numel(z) >= 2 && all(isfinite(z(1:2)));

    % ---------- Initialization from GNSS ----------
    if ~isfield(public_vars, 'gnss_initialized') || ~public_vars.gnss_initialized

        if gnss_ok
            mu = [z(1); z(2); 0];
            sigma = diag([0.5, 0.5, public_vars.initial_theta_variance]);
        end

        return;
    end

    % ---------- Prediction ----------
    u = public_vars.motion_vector(:);

    if numel(u) ~= 2 || any(~isfinite(u))
        u = [0; 0];
    end

    [mu, sigma] = ekf_predict( ...
        mu, ...
        sigma, ...
        u, ...
        public_vars.kf, ...
        read_only_vars.sampling_period);

    % ---------- GNSS correction ----------
    if gnss_ok
        [mu, sigma] = kf_measure(mu, sigma, z(1:2), public_vars.kf);
    end
end