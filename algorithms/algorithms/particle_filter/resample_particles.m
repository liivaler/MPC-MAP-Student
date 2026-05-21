function new_particles = resample_particles(particles, weights)
% RESAMPLE_PARTICLES
% Systematic resampling.

    N = size(particles,1);
    new_particles = zeros(size(particles));

    weights = weights(:);
    weights(~isfinite(weights)) = 0;
    weights = weights + 1e-12;
    weights = weights / sum(weights);

    cdf = cumsum(weights);
    cdf(end) = 1;

    step = 1/N;
    u = rand*step;
    idx = 1;

    for i = 1:N
        threshold = u + (i-1)*step;
        while cdf(idx) < threshold
            idx = idx + 1;
        end
        new_particles(i,:) = particles(idx,:);
    end
end
