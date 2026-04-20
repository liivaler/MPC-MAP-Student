function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
% KF_MEASURE
% Linear correction step for GNSS measurement z = [x; y]

    C = kf.C;
    Q = kf.Q;

    % Innovation
    y = z - C*mu;

    % Innovation covariance
    S = C*sigma*C' + Q;

    % Kalman gain
    K = sigma*C'/S;

    % State update
    new_mu = mu + K*y;

    % Covariance update
    I = eye(size(sigma));
    new_sigma = (I - K*C)*sigma;

    % Normalize angle
    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
end