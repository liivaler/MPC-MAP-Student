function public_vars = init_particle_filter(read_only_vars, public_vars)
% INIT_PARTICLE_FILTER
% Scan-matched global initialization for indoor / mixed maps.
%
% Does NOT use mocap.
% Uses only:
%   - map
%   - lidar_distances
%   - lidar_config
%   - gnss_position only to decide whether to prefer GNSS denied area

    % ============================================================
    % 1) Number of final particles
    % ============================================================
    if isfield(read_only_vars, 'max_particles') && read_only_vars.max_particles > 0
        N = read_only_vars.max_particles;
    else
        N = 1000;
    end

    public_vars.Npf = N;
    public_vars.particles = zeros(N,3);
    public_vars.weights = ones(N,1) / N;

    public_vars.pf_initialized = true;
    public_vars.pf_lock_counter = 0;

    % ============================================================
    % 2) Decide whether to prefer GNSS denied / indoor area
    % ============================================================
    z = read_only_vars.gnss_position(:);
    gnss_ok = numel(z) >= 2 && all(isfinite(z(1:2)));

    has_denied = isfield(read_only_vars.map, 'gnss_denied') && ...
                 ~isempty(read_only_vars.map.gnss_denied);

    % Mixed-map fix:
    % If GNSS is not available and map has gnss_denied polygon,
    % initialize PF mainly inside that polygon.
    prefer_denied = ~gnss_ok && has_denied;

    % ============================================================
    % 3) If lidar is available, use scan-matched initialization
    % ============================================================
    has_lidar = isfield(read_only_vars, 'lidar_distances') && ...
                ~isempty(read_only_vars.lidar_distances) && ...
                all(isfinite(read_only_vars.lidar_distances(:))) && ...
                isfield(read_only_vars, 'lidar_config') && ...
                ~isempty(read_only_vars.lidar_config);

    if has_lidar

        % More candidates than final particles.
        % Keep this local only, not in public_vars.
        Ncand = max(8000, 8*N);

        candidates = zeros(Ncand, 3);

        for i = 1:Ncand
            candidates(i,:) = random_free_particle(read_only_vars, 0.06, prefer_denied);
        end

        % Compute predicted lidar for all candidates
        M = length(read_only_vars.lidar_config);
        measurements = zeros(Ncand, M);

        for i = 1:Ncand
            measurements(i,:) = compute_lidar_measurement( ...
                read_only_vars.map, ...
                candidates(i,:), ...
                read_only_vars.lidar_config);
        end

        % Weight candidates using current real lidar
        cand_weights = weight_particles( ...
            measurements, ...
            read_only_vars.lidar_distances);

        cand_weights = cand_weights(:);
        cand_weights(~isfinite(cand_weights)) = 0;
        cand_weights = cand_weights + 1e-15;
        cand_weights = cand_weights / sum(cand_weights);

        % Resample exactly N particles from good candidates
        idx = systematic_resample_indices(cand_weights, N);

        public_vars.particles = candidates(idx,:);

        % Small roughening so particles are not identical
        for i = 1:N

            p_old = public_vars.particles(i,:);

            p_try = p_old;
            p_try(1) = p_try(1) + 0.03*randn;
            p_try(2) = p_try(2) + 0.03*randn;
            p_try(3) = wrap_angle(p_try(3) + 0.05*randn);

            if is_pose_safe(p_try, read_only_vars, 0.04)
                public_vars.particles(i,:) = p_try;
            else
                public_vars.particles(i,:) = p_old;
            end
        end

        public_vars.weights = ones(N,1) / N;
        return;
    end

    % ============================================================
    % 4) Fallback: random free initialization
    % ============================================================
    for i = 1:N
        public_vars.particles(i,:) = random_free_particle(read_only_vars, 0.06, prefer_denied);
    end

    public_vars.weights = ones(N,1) / N;
end


% ========================================================================
% Helper functions
% ========================================================================

function idx = systematic_resample_indices(weights, N)

    weights = weights(:);
    weights(~isfinite(weights)) = 0;
    weights = weights + 1e-15;
    weights = weights / sum(weights);

    cdf = cumsum(weights);
    cdf(end) = 1;

    idx = zeros(N,1);

    step = 1/N;
    u = rand * step;

    j = 1;

    for i = 1:N
        threshold = u + (i-1)*step;

        while cdf(j) < threshold
            j = j + 1;
        end

        idx(i) = j;
    end
end


function p = random_free_particle(read_only_vars, clearance, prefer_denied)
% RANDOM_FREE_PARTICLE
% If prefer_denied == true and gnss_denied exists, sample mainly inside
% GNSS denied area. Otherwise sample globally.

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

    % ============================================================
    % 1) Preferred sampling in GNSS denied area
    % ============================================================
    if prefer_denied && has_denied

        for t = 1:5000

            x = xmin + (xmax - xmin) * rand;
            y = ymin + (ymax - ymin) * rand;
            theta = -pi + 2*pi*rand;

            p_try = [x y theta];

            if ~inside_gnss_denied(x, y, read_only_vars.map.gnss_denied)
                continue;
            end

            if is_pose_safe(p_try, read_only_vars, clearance)
                p = p_try;
                return;
            end
        end
    end

    % ============================================================
    % 2) Global fallback sampling
    % ============================================================
    for t = 1:4000

        x = xmin + (xmax - xmin) * rand;
        y = ymin + (ymax - ymin) * rand;
        theta = -pi + 2*pi*rand;

        p_try = [x y theta];

        if is_pose_safe(p_try, read_only_vars, clearance)
            p = p_try;
            return;
        end
    end

    % Last fallback
    p = [ ...
        0.5*(xmin + xmax), ...
        0.5*(ymin + ymax), ...
        -pi + 2*pi*rand ...
    ];
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

    if ~isfield(read_only_vars.map, 'walls') || isempty(read_only_vars.map.walls)
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
        d = hypot(px - x1, py - y1);
        return;
    end

    t = (wx*vx + wy*vy) / len2;
    t = max(0, min(1, t));

    proj_x = x1 + t*vx;
    proj_y = y1 + t*vy;

    d = hypot(px - proj_x, py - proj_y);
end


function a = wrap_angle(a)

    a = atan2(sin(a), cos(a));
end