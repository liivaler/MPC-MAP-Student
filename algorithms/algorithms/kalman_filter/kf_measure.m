function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
% KF_MEASURE
% Linear correction step for GNSS measurement:

    C = kf.C;
    R = kf.R;

    z = z(:);

    % Innovation
    y = z - C * mu;

    % Innovation covariance
    S = C * sigma * C' + R;

    % Kalman gain
    K = sigma * C' / S;

    % State update
    new_mu = mu + K * y;

    % Stable covariance update
    I = eye(size(sigma));
    new_sigma = (I - K*C) * sigma * (I - K*C)' + K * R * K';

    % Normalize angle
    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

    % Numerical symmetry
    new_sigma = 0.5 * (new_sigma + new_sigma');
end