function [new_path] = smooth_path(old_path, read_only_vars)
% SMOOTH_PATH - iterative smoothing with safety check

    if isempty(old_path) || size(old_path,1) < 3
        new_path = old_path;
        return;
    end

    candidate = old_path;

    alpha = 0.1;      % keeps path close to original
    beta  = 0.2;      % smoothing strength
    iterations = 25;

    for it = 1:iterations
        for i = 2:size(old_path,1)-1
            candidate(i,:) = candidate(i,:) ...
                + alpha * (old_path(i,:) - candidate(i,:)) ...
                + beta  * (candidate(i-1,:) + candidate(i+1,:) - 2*candidate(i,:));
        end
    end

    % Use smoothed path only if it is still safe
    if is_path_safe(candidate, read_only_vars)
        new_path = candidate;
    else
        disp('Smoothed path is not safe. Using original A* path.');
        new_path = old_path;
    end
end


function safe = is_path_safe(path, read_only_vars)

    safe = true;

    walls = read_only_vars.map.walls;
    limits = read_only_vars.map.limits;

    clearance = 0.20;

    xmin = limits(1);
    ymin = limits(2);
    xmax = limits(3);
    ymax = limits(4);

    for i = 1:size(path,1)-1

        p1 = path(i,:);
        p2 = path(i+1,:);

        dist = norm(p2 - p1);
        n = max(2, ceil(dist / 0.10));

        for k = 0:n

            s = k / n;
            p = p1 + s * (p2 - p1);

            x = p(1);
            y = p(2);

            if x < xmin || x > xmax || y < ymin || y > ymax
                safe = false;
                return;
            end

            for j = 1:size(walls,1)

                d = point_to_segment_distance( ...
                    x, y, ...
                    walls(j,1), walls(j,2), ...
                    walls(j,3), walls(j,4));

                if d < clearance
                    safe = false;
                    return;
                end
            end
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
    t = max(0, min(1, t));

    proj_x = x1 + t*vx;
    proj_y = y1 + t*vy;

    d = hypot(px-proj_x, py-proj_y);
end