function [weights] = weight_particles(particle_measurements, lidar_distances)

    N = size(particle_measurements,1);
    weights = zeros(N,1);

    for i = 1:N
        z_pred = particle_measurements(i,:);
        z_real = lidar_distances;

        err = mean(abs(z_real - z_pred));

        weights(i) = exp(-3 * err);
    end

    s = sum(weights);

    if s > 0
        weights = weights / s;
    else
        weights = ones(N,1)/N;
    end
end