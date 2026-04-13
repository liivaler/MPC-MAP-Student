function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
% PREDICT_POSE
% Differential drive motion model with noise.

    vR = motion_vector(1);
    vL = motion_vector(2);

    L = read_only_vars.agent_drive.interwheel_dist;
    dt = read_only_vars.sampling_period;

    v = (vR + vL)/2;
    w = (vR - vL)/L;

    x = old_pose(1);
    y = old_pose(2);
    theta = old_pose(3);

    % základní pohybový model
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + w*dt;

    % náhodný šum
    x = x + 0.05*randn;
    y = y + 0.05*randn;
    theta = theta + 0.05*randn;

    % normalizace úhlu
    theta = atan2(sin(theta), cos(theta));

    new_pose = [x, y, theta];
end