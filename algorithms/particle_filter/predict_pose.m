function new_pose = predict_pose(old_pose, motion_vector, read_only_vars)
% PREDICT_POSE
% Differential-drive prediction for one particle. motion_vector = [vR vL].

    if numel(old_pose) < 3 || any(~isfinite(old_pose))
        old_pose = [0 0 0];
    end

    if numel(motion_vector) < 2 || any(~isfinite(motion_vector))
        motion_vector = [0 0];
    end

    vR = motion_vector(1);
    vL = motion_vector(2);

    L = read_only_vars.agent_drive.interwheel_dist;
    dt = read_only_vars.sampling_period;

    v = 0.5*(vR + vL);
    w = (vR - vL)/L;

    x = old_pose(1);
    y = old_pose(2);
    th = atan2(sin(old_pose(3)), cos(old_pose(3)));

    x = x + v*cos(th)*dt;
    y = y + v*sin(th)*dt;
    th = th + w*dt;

    % Noise: enough for recovery, but not so much that indoor pose drifts.
    motion_mag = abs(v)*dt + abs(w)*dt;
    sigma_xy = 0.004 + 0.010*motion_mag;
    sigma_th = 0.008 + 0.020*motion_mag;

    if abs(v) < 0.02 && abs(w) > 0.05
        sigma_xy = 0.003;
        sigma_th = sigma_th + 0.008;
    end

    x = x + sigma_xy*randn;
    y = y + sigma_xy*randn;
    th = atan2(sin(th + sigma_th*randn), cos(th + sigma_th*randn));

    new_pose = [x y th];
end
