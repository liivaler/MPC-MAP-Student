function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
% EKF_PREDICT
% EKF prediction for differential drive robot.
%
% State:
%   x = [x; y; theta]
%
% Input:
%   u = [vR; vL]

    if numel(u) < 2 || any(~isfinite(u))
        u = [0; 0];
    end

    vR = u(1);
    vL = u(2);

    L = kf.L;
    dt = sampling_period;

    v = (vR + vL) / 2;
    w = (vR - vL) / L;

    x = mu(1);
    y = mu(2);
    theta = mu(3);

    new_mu = zeros(3,1);

    new_mu(1) = x + v*cos(theta)*dt;
    new_mu(2) = y + v*sin(theta)*dt;
    new_mu(3) = theta + w*dt;

    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

    G = [1 0 -v*sin(theta)*dt;
         0 1  v*cos(theta)*dt;
         0 0  1];

    % Q = process noise
    new_sigma = G * sigma * G' + kf.Q;

    % numerical symmetry
    new_sigma = 0.5 * (new_sigma + new_sigma');
end