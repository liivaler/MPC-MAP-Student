function [new_particles] = resample_particles(particles, weights)
% RESAMPLE_PARTICLES
% Multinomial resampling without randsample.

    N = size(particles,1);
    new_particles = zeros(size(particles));

    % kumulativní součet vah
    cdf = cumsum(weights);
    cdf(end) = 1;   % jistota kvůli numerice

    for i = 1:N
        r = rand;
        idx = find(cdf >= r, 1, 'first');
        new_particles(i,:) = particles(idx,:);
    end
end