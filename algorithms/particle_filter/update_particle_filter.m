function [particles, weights] = update_particle_filter(read_only_vars, public_vars)
% UPDATE_PARTICLE_FILTER
% One particle filter iteration:
% 1) prediction
% 2) expected lidar measurements
% 3) weighting
% 4) conditional resampling

    particles = public_vars.particles;
    N = size(particles,1);

    % ------------------------------------------------------------
    % 1) Prediction
    % ------------------------------------------------------------
    for i = 1:N
        particles(i,:) = predict_pose( ...
            particles(i,:), ...
            public_vars.motion_vector, ...
            read_only_vars);
    end

    % ------------------------------------------------------------
    % 2) Expected lidar measurements for all particles
    % ------------------------------------------------------------
    particle_measurements = zeros(N, length(read_only_vars.lidar_config));

    for i = 1:N
        particle_measurements(i,:) = compute_lidar_measurement( ...
            read_only_vars.map, ...
            particles(i,:), ...
            read_only_vars.lidar_config);
    end

    % ------------------------------------------------------------
    % 3) Weighting
    % ------------------------------------------------------------
    weights = weight_particles( ...
        particle_measurements, ...
        read_only_vars.lidar_distances);

    % ------------------------------------------------------------
    % 4) Effective sample size
    % ------------------------------------------------------------
    Neff = 1 / sum(weights.^2);

    % ------------------------------------------------------------
    % 5) Resample only if needed
    % ------------------------------------------------------------
    if Neff < 0.5 * N
        particles = resample_particles(particles, weights);

        % small fraction of random particles
        num_random = round(0.05 * N);

        xmin = read_only_vars.map.limits(1);
        xmax = read_only_vars.map.limits(3);
        ymin = read_only_vars.map.limits(2);
        ymax = read_only_vars.map.limits(4);

        inserted = 0;
        while inserted < num_random
            x = xmin + (xmax - xmin) * rand;
            y = ymin + (ymax - ymin) * rand;
            theta = 2*pi*rand;

            % keep only particles inside GNSS denied region
            inside = false;
            for k = 1:size(read_only_vars.map.gnss_denied,1)
                xv = read_only_vars.map.gnss_denied(k,1:2:end);
                yv = read_only_vars.map.gnss_denied(k,2:2:end);
                if inpolygon(x, y, xv, yv)
                    inside = true;
                end
            end

            if inside
                inserted = inserted + 1;
                particles(inserted,:) = [x, y, theta];
            end
        end
    end
end