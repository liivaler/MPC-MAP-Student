function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
% EKF_PREDICT
% EKF prediction for differential drive robot
%
% State: x = [x; y; theta]
% Input: u = [vR; vL]

    vR = u(1);
    vL = u(2);

    L = kf.L;

    % Convert wheel speeds to linear/angular speed
    v = (vR + vL)/2;
    w = (vR - vL)/L;

    x = mu(1);
    y = mu(2);
    theta = mu(3);
    dt = sampling_period;

    % Nonlinear state prediction
    new_mu = zeros(3,1);
    new_mu(1) = x + v*cos(theta)*dt;
    new_mu(2) = y + v*sin(theta)*dt;
    new_mu(3) = theta + w*dt;

    % Normalize angle
    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

    % Jacobian G = dg/dx
    G = [1 0 -v*sin(theta)*dt;
         0 1  v*cos(theta)*dt;
         0 0  1];

    % Covariance prediction
    new_sigma = G * sigma * G' + kf.R;
end