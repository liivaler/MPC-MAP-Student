map_name = 'maps/indoor_1.txt';

% náhodný bezpečný start ve volném prostoru
tmp_map = load_map(map_name);
start_position = random_start_pose(tmp_map, 0.35);
function p = random_start_pose(map, clearance)

    limits = map.limits;

    xmin = limits(1);
    ymin = limits(2);
    xmax = limits(3);
    ymax = limits(4);

    for k = 1:10000
        x = xmin + (xmax - xmin) * rand;
        y = ymin + (ymax - ymin) * rand;
        th = -pi + 2*pi*rand;

        if is_pose_safe([x y th], map, clearance)
            p = [x y th];
            return;
        end
    end

    % fallback
    p = [3, 1.5, pi/3];
end


function ok = is_pose_safe(p, map, clearance)

    x = p(1);
    y = p(2);

    limits = map.limits;

    ok = x > limits(1) && x < limits(3) && ...
         y > limits(2) && y < limits(4);

    if ~ok
        return;
    end

    walls = map.walls;

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