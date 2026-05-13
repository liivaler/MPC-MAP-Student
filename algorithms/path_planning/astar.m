function best_path = astar(read_only_vars, public_vars)
% ASTAR
% Toolbox-free A* path planner for final project.

    occ_map = double(read_only_vars.discrete_map.map ~= 0);

    limits = read_only_vars.map.limits;
    walls  = read_only_vars.map.walls;

    start_xy = public_vars.estimated_pose(1:2);
    start_xy = start_xy(:)';

    if numel(start_xy) ~= 2 || any(~isfinite(start_xy))
        best_path = [];
        return;
    end

    goal_xy = read_only_vars.map.goal(1:2);
    goal_xy = goal_xy(:)';

    clearances = [0.22 0.2 0.19 0.18 0.16];

    best_path = [];

    for i = 1:numel(clearances)
        clearance = clearances(i);

        path = run_astar_on_map(occ_map, limits, start_xy, goal_xy, clearance);

        if ~isempty(path) && is_path_safe(path, walls, limits, clearance)
            best_path = path;
            return;
        end
    end
end


function path = run_astar_on_map(occ_map, limits, start_xy, goal_xy, clearance)

    [rows, cols] = size(occ_map);

    xmin = limits(1); ymin = limits(2);
    xmax = limits(3); ymax = limits(4);

    dx = (xmax - xmin) / (cols - 1);
    dy = (ymax - ymin) / (rows - 1);
    cell_size = min(dx, dy);

    n_clear = max(1, ceil(clearance / cell_size));
    occ_map = inflate_obstacles(occ_map, n_clear);

    [sr, sc] = world_to_grid(start_xy(1), start_xy(2), limits, rows, cols);
    [gr, gc] = world_to_grid(goal_xy(1), goal_xy(2), limits, rows, cols);

    [sr, sc] = nearest_free(occ_map, sr, sc);
    [gr, gc] = nearest_free(occ_map, gr, gc);

    g = inf(rows, cols);
    f = inf(rows, cols);

    parent_r = zeros(rows, cols);
    parent_c = zeros(rows, cols);

    open = false(rows, cols);
    closed = false(rows, cols);

    g(sr,sc) = 0;
    f(sr,sc) = hypot(sr-gr, sc-gc);
    open(sr,sc) = true;

    nbrs = [
        -1  0
         1  0
         0 -1
         0  1
        -1 -1
        -1  1
         1 -1
         1  1
    ];

    found = false;

    while any(open(:))
        open_idx = find(open);
        [~, best_i] = min(f(open_idx));
        current = open_idx(best_i);

        [r, c] = ind2sub([rows, cols], current);

        if r == gr && c == gc
            found = true;
            break;
        end

        open(r,c) = false;
        closed(r,c) = true;

        for k = 1:size(nbrs,1)
            rr = r + nbrs(k,1);
            cc = c + nbrs(k,2);

            if rr < 1 || rr > rows || cc < 1 || cc > cols
                continue;
            end

            if closed(rr,cc) || occ_map(rr,cc) == 1
                continue;
            end

            if abs(nbrs(k,1)) == 1 && abs(nbrs(k,2)) == 1
                if occ_map(r,cc) == 1 || occ_map(rr,c) == 1
                    continue;
                end
            end

            tentative_g = g(r,c) + hypot(nbrs(k,1), nbrs(k,2));

            if tentative_g < g(rr,cc)
                parent_r(rr,cc) = r;
                parent_c(rr,cc) = c;

                g(rr,cc) = tentative_g;
                f(rr,cc) = tentative_g + hypot(rr-gr, cc-gc);

                open(rr,cc) = true;
            end
        end
    end

    if ~found
        path = [];
        return;
    end

    grid_path = [gr gc];

    r = gr;
    c = gc;

    while ~(r == sr && c == sc)
        pr = parent_r(r,c);
        pc = parent_c(r,c);

        if pr == 0 && pc == 0
            path = [];
            return;
        end

        grid_path = [pr pc; grid_path];

        r = pr;
        c = pc;
    end

    path = zeros(size(grid_path,1), 2);

    for i = 1:size(grid_path,1)
        [x, y] = grid_to_world(grid_path(i,1), grid_path(i,2), limits, rows, cols);
        path(i,:) = [x, y];
    end
end


function inflated_map = inflate_obstacles(occ_map, n_clear)

    inflated_map = occ_map;

    [rows, cols] = size(occ_map);
    [obs_r, obs_c] = find(occ_map == 1);

    for k = 1:length(obs_r)
        r0 = obs_r(k);
        c0 = obs_c(k);

        rmin = max(1, r0 - n_clear);
        rmax = min(rows, r0 + n_clear);
        cmin = max(1, c0 - n_clear);
        cmax = min(cols, c0 + n_clear);

        for r = rmin:rmax
            for c = cmin:cmax
                if hypot(r-r0, c-c0) <= n_clear
                    inflated_map(r,c) = 1;
                end
            end
        end
    end
end


function [r, c] = world_to_grid(x, y, limits, rows, cols)

    xmin = limits(1); ymin = limits(2);
    xmax = limits(3); ymax = limits(4);

    c = round((x - xmin) / (xmax - xmin) * (cols - 1)) + 1;
    r = round((y - ymin) / (ymax - ymin) * (rows - 1)) + 1;

    c = min(max(c,1),cols);
    r = min(max(r,1),rows);
end


function [x, y] = grid_to_world(r, c, limits, rows, cols)

    xmin = limits(1); ymin = limits(2);
    xmax = limits(3); ymax = limits(4);

    x = xmin + (c - 1) / (cols - 1) * (xmax - xmin);
    y = ymin + (r - 1) / (rows - 1) * (ymax - ymin);
end


function [r_free, c_free] = nearest_free(map, r0, c0)

    [rows, cols] = size(map);

    r0 = min(max(r0,1),rows);
    c0 = min(max(c0,1),cols);

    if map(r0,c0) == 0
        r_free = r0;
        c_free = c0;
        return;
    end

    best_dist = inf;
    r_free = r0;
    c_free = c0;

    for r = 1:rows
        for c = 1:cols
            if map(r,c) == 0
                d = hypot(r-r0, c-c0);
                if d < best_dist
                    best_dist = d;
                    r_free = r;
                    c_free = c;
                end
            end
        end
    end
end


function safe = is_path_safe(path, walls, limits, clearance)

    safe = true;

    xmin = limits(1); ymin = limits(2);
    xmax = limits(3); ymax = limits(4);

    for i = 1:size(path,1)-1
        p1 = path(i,:);
        p2 = path(i+1,:);

        dist = norm(p2-p1);
        n = max(2, ceil(dist / 0.05));

        for k = 0:n
            s = k / n;
            p = p1 + s * (p2-p1);

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