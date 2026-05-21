function measurement = compute_lidar_measurement(map, pose, lidar_config)
% COMPUTE_LIDAR_MEASUREMENT
% Ideal lidar distances from pose = [x y theta].
% lidar_config contains relative beam angles in robot frame.

    N = length(lidar_config);
    measurement = zeros(1,N);

    if numel(pose) < 3 || any(~isfinite(pose))
        measurement(:) = 10;
        return;
    end

    origin = pose(1:2);
    theta = atan2(sin(pose(3)), cos(pose(3)));

    % The simulator lidar can return values > 8 m, so 5 m is wrong here.
    max_range = 10;

    if ~isfield(map,'walls') || isempty(map.walls)
        measurement(:) = max_range;
        return;
    end

    for i = 1:N
        dir = theta + lidar_config(i);
        dir = atan2(sin(dir), cos(dir));

        intersections = ray_cast(origin, map.walls, dir);

        if isempty(intersections)
            measurement(i) = max_range;
            continue;
        end

        valid = isfinite(intersections(:,1)) & isfinite(intersections(:,2));
        intersections = intersections(valid,:);

        if isempty(intersections)
            measurement(i) = max_range;
            continue;
        end

        dists = hypot(intersections(:,1)-origin(1), intersections(:,2)-origin(2));
        dists = dists(dists > 0.05 & dists <= max_range);

        if isempty(dists)
            measurement(i) = max_range;
        else
            measurement(i) = min(dists);
        end
    end
end
