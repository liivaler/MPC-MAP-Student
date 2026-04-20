function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
% UPDATE_KALMAN_FILTER

    mu = public_vars.mu;
    sigma = public_vars.sigma;

    % Prediction input = wheel velocities from motion controller
    u = public_vars.motion_vector(:);

    % I. Prediction
    [mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

    % II. Measurement correction with GNSS
    z = read_only_vars.gnss_position(:);

    if all(isfinite(z))
        [mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);
    end
end