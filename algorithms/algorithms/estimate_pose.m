function estimated_pose = estimate_pose(public_vars)
% ESTIMATE_POSE
% Estimate pose from the strongest local particle cluster.
%
% Works with PF after resampling, where weights may be uniform.
% Therefore the main information is particle density, not max weight.

    particles = public_vars.particles;
    weights = public_vars.weights(:);

    N = size(particles,1);

    if N == 0 || isempty(weights) || numel(weights) ~= N
        estimated_pose = [NaN NaN NaN];
        return;
    end

    weights(~isfinite(weights)) = 0;

    if sum(weights) <= 0
        weights = ones(N,1) / N;
    else
        weights = weights + 1e-15;
        weights = weights / sum(weights);
    end

    % ============================================================
    % 1) Find strongest local cluster
    % ============================================================
    cluster_radius = 0.70;

    % Deterministic candidates from whole particle cloud.
    Kcand = min(160, N);
    candidates = unique(round(linspace(1, N, Kcand)));

    best_score = -inf;
    best_idx = candidates(1);

    for k = 1:numel(candidates)

        idx = candidates(k);

        cx = particles(idx,1);
        cy = particles(idx,2);

        d = hypot(particles(:,1) - cx, particles(:,2) - cy);
        local = d < cluster_radius;

        cnt = sum(local);

        if cnt < 10
            continue;
        end

        % Density score.
        % If weights are uniform, this is just number of nearby particles.
        % If weights still contain information, it also uses it.
        score = cnt / N + sum(weights(local));

        if score > best_score
            best_score = score;
            best_idx = idx;
        end
    end

    % ============================================================
    % 2) Estimate pose only from chosen local cluster
    % ============================================================
    cx = particles(best_idx,1);
    cy = particles(best_idx,2);

    d = hypot(particles(:,1) - cx, particles(:,2) - cy);
    local = d < cluster_radius;

    if sum(local) < 10
        estimated_pose = particles(best_idx,:);
        estimated_pose(3) = wrap_angle(estimated_pose(3));
        return;
    end

    P = particles(local,:);
    W = weights(local);

    W = W + 1e-15;
    W = W / sum(W);

    x = sum(P(:,1) .* W);
    y = sum(P(:,2) .* W);

    c = sum(cos(P(:,3)) .* W);
    s = sum(sin(P(:,3)) .* W);

    theta = atan2(s, c);

    estimated_pose = [x y theta];
    estimated_pose(3) = wrap_angle(estimated_pose(3));
end


function a = wrap_angle(a)
    a = atan2(sin(a), cos(a));
end