function [particles, weights] = update_particle_filter(read_only_vars, public_vars)
% UPDATE_PARTICLE_FILTER
% PF update: predict -> inject recovery -> weight -> resample -> roughen -> reweight.
% Does NOT use mocap.
%
% Mixed map fix:
% When GNSS is not available and map.gnss_denied exists, random recovery
% particles are injected mainly inside the GNSS denied / indoor area.

    particles = public_vars.particles;
    N = size(particles,1);

    if N == 0
        weights = [];
        return;
    end

    % ------------------------------------------------------------
    % 0) Decide whether random particles should prefer GNSS denied
    % ------------------------------------------------------------
    z = read_only_vars.gnss_position(:);
    gnss_ok = numel(z) >= 2 && all(isfinite(z(1:2)));

    has_denied = isfield(read_only_vars.map, 'gnss_denied') && ...
                 ~isempty(read_only_vars.map.gnss_denied);

    prefer_denied = ~gnss_ok && has_denied;

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
    % 2) Small recovery injection before weighting
    % ------------------------------------------------------------
    lock_count = 0;
    if isfield(public_vars,'pf_lock_counter')
        lock_count = public_vars.pf_lock_counter;
    end

    if lock_count < 10
        frac_random = 0.02;
    elseif lock_count < 40
        frac_random = 0.01;
    else
        frac_random = 0.002;
    end

    % If we started globally without GNSS, allow a bit more exploration.
    if isfield(public_vars,'pf_global_search') && public_vars.pf_global_search && lock_count < 5
        frac_random = 0.04;
    end

    num_random = round(frac_random * N);

    if num_random > 0
        idxs = randperm(N, num_random);

        for k = 1:num_random
            particles(idxs(k),:) = random_free_particle( ...
                read_only_vars, ...
                0.10, ...
                prefer_denied);
        end
    end

    % ------------------------------------------------------------
    % 3) Weight particles by lidar
    % ------------------------------------------------------------
    M = length(read_only_vars.lidar_config);
    particle_measurements = zeros(N, M);

    for i = 1:N
        particle_measurements(i,:) = compute_lidar_measurement( ...
            read_only_vars.map, ...
            particles(i,:), ...
            read_only_vars.lidar_config);
    end

    weights = weight_particles(particle_measurements, read_only_vars.lidar_distances);
    weights = normalize_weights(weights, N);

    % ------------------------------------------------------------
    % 4) Resample every iteration
    % ------------------------------------------------------------
    particles = resample_particles(particles, weights);

    % ------------------------------------------------------------
    % 5) Roughen after resampling, not too much
    % ------------------------------------------------------------
    particles = roughen_particles(particles, read_only_vars, lock_count);

    % ------------------------------------------------------------
    % 6) Keep uniform weights after resampling + roughening
    % ------------------------------------------------------------
    % Faster version:
    % The pose estimator below is density-based, so we do not need
    % a second lidar simulation pass in the same iteration.
    weights = ones(N,1) / N;
end


function weights = normalize_weights(weights, N)

    weights = weights(:);

    if isempty(weights) || numel(weights) ~= N || any(~isfinite(weights)) || sum(weights) <= 0
        weights = ones(N,1) / N;
        return;
    end

    weights = weights + 1e-12;
    weights = weights / sum(weights);
end


function particles = roughen_particles(particles, read_only_vars, lock_count)

    if lock_count < 10
        sx = 0.015; sy = 0.015; sth = 0.025;
    elseif lock_count < 40
        sx = 0.008; sy = 0.008; sth = 0.015;
    else
        sx = 0.003; sy = 0.003; sth = 0.008;
    end

    N = size(particles,1);

    for i = 1:N

        p_old = particles(i,:);
        p_try = p_old;

        p_try(1) = p_try(1) + sx * randn;
        p_try(2) = p_try(2) + sy * randn;
        p_try(3) = wrap_angle(p_try(3) + sth * randn);

        if is_pose_safe(p_try, read_only_vars, 0.06)
            particles(i,:) = p_try;
        else
            particles(i,:) = p_old;
        end
    end
end


function p = random_free_particle(read_only_vars, clearance, prefer_denied)
% RANDOM_FREE_PARTICLE
% If prefer_denied == true and map.gnss_denied exists, sample mainly inside
% GNSS denied / indoor area. Otherwise sample globally.

    if nargin < 3
        prefer_denied = false;
    end

    limits = read_only_vars.map.limits;

    xmin = limits(1);
    ymin = limits(2);
    xmax = limits(3);
    ymax = limits(4);

    has_denied = isfield(read_only_vars.map, 'gnss_denied') && ...
                 ~isempty(read_only_vars.map.gnss_denied);

    % ------------------------------------------------------------
    % 1) Prefer GNSS denied / indoor area if requested
    % ------------------------------------------------------------
    if prefer_denied && has_denied

        for t = 1:4000

            x = xmin + (xmax - xmin) * rand;
            y = ymin + (ymax - ymin) * rand;
            th = -pi + 2*pi*rand;

            p_try = [x y th];

            if ~inside_gnss_denied(x, y, read_only_vars.map.gnss_denied)
                continue;
            end

            if is_pose_safe(p_try, read_only_vars, clearance)
                p = p_try;
                return;
            end
        end
    end

    % ------------------------------------------------------------
    % 2) Global fallback
    % ------------------------------------------------------------
    for t = 1:3000

        x = xmin + (xmax - xmin) * rand;
        y = ymin + (ymax - ymin) * rand;
        th = -pi + 2*pi*rand;

        p_try = [x y th];

        if is_pose_safe(p_try, read_only_vars, clearance)
            p = p_try;
            return;
        end
    end

    p = [0.5*(xmin+xmax), 0.5*(ymin+ymax), -pi + 2*pi*rand];
end


function inside = inside_gnss_denied(x, y, gnss_denied)

    inside = false;

    for k = 1:size(gnss_denied,1)

        xv = gnss_denied(k,1:2:end);
        yv = gnss_denied(k,2:2:end);

        if inpolygon(x, y, xv, yv)
            inside = true;
            return;
        end
    end
end


function ok = is_pose_safe(p, read_only_vars, clearance)

    x = p(1);
    y = p(2);

    limits = read_only_vars.map.limits;

    ok = x > limits(1) && x < limits(3) && ...
         y > limits(2) && y < limits(4);

    if ~ok
        return;
    end

    if ~isfield(read_only_vars.map,'walls') || isempty(read_only_vars.map.walls)
        return;
    end

    walls = read_only_vars.map.walls;

    for j = 1:size(walls,1)

        d = point_to_segment_distance( ...
            x, y, ...
            walls(j,1), walls(j,2), ...
            walls(j,3), walls(j,4));

        if d < clearance
            ok = false;
            return;
        end
    end
end


function d = point_to_segment_distance(px, py, x1, y1, x2, y2)

    vx = x2 - x1;
    vy = y2 - y1;

    wx = px - x1;
    wy = py - y1;

    len2 = vx*vx + vy*vy;

    if len2 == 0
        d = hypot(px-x1, py-y1);
        return;
    end

    t = (wx*vx + wy*vy) / len2;
    t = max(0, min(1,t));

    proj_x = x1 + t*vx;
    proj_y = y1 + t*vy;

    d = hypot(px-proj_x, py-proj_y);
end


function a = wrap_angle(a)

    a = atan2(sin(a), cos(a));
end