function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
% COMPUTE_LIDAR_MEASUREMENT
% Returns ideal lidar distances for a given pose.

    N = length(lidar_config);
    measurement = zeros(1, N);

    origin = pose(1:2);

    for i = 1:N
        dir = pose(3) + lidar_config(i);

        intersections = ray_cast(origin, map.walls, dir);

        % ray_cast vrací i NaN řádky -> odfiltrujeme
        valid = ~isnan(intersections(:,1)) & ~isnan(intersections(:,2));
        intersections = intersections(valid,:);

        if isempty(intersections)
            measurement(i) = 10;   % fallback maximum range
        else
            dists = sqrt( ...
                (intersections(:,1) - origin(1)).^2 + ...
                (intersections(:,2) - origin(2)).^2 );

            measurement(i) = min(dists);
        end
    end
end