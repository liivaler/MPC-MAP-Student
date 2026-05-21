function weights = weight_particles(particle_measurements, lidar_distances)
% WEIGHT_PARTICLES
% Lidar likelihood. Uses log-domain squared error, like the functional PF,
% but with fixed parameters and 10 m lidar range.

    N = size(particle_measurements,1);
    if N == 0
        weights = [];
        return;
    end

    z_real = lidar_distances(:)';
    M = min(size(particle_measurements,2), numel(z_real));

    if M < 3
        weights = ones(N,1)/N;
        return;
    end

    z_real = z_real(1:M);
    Z = particle_measurements(:,1:M);

    max_range = 10.0;
    min_range = 0.03;

    z_real(~isfinite(z_real) | z_real <= 0) = max_range;
    Z(~isfinite(Z)) = max_range;

    z_real = max(min_range, min(max_range, z_real));
    Z = max(min_range, min(max_range, Z));

    sigma = 0.35;
    max_err = 3.0;

    logw = zeros(N,1);
    for i = 1:N
        err = z_real - Z(i,:);
        err = max(-max_err, min(max_err, err));
        logw(i) = -sum(err.^2) / (2*sigma^2);
    end

    m = max(logw);
    if ~isfinite(m)
        weights = ones(N,1)/N;
        return;
    end

    weights = exp(logw - m);
    weights = weights + 1e-15;
    weights = weights / sum(weights);
end
