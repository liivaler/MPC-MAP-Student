function path = plan_path(read_only_vars, public_vars)
% Week 6 path planning wrapper.
% Uses A* and optional smoothing.

    raw_path = astar(read_only_vars, public_vars);

    if isempty(raw_path)
        path = [];
        return;
    end

    path = raw_path;

    try
        smoothed = smooth_path(raw_path, read_only_vars);

        if ~isempty(smoothed) && size(smoothed,1) >= 2
            path = smoothed;
        end
    catch
        path = raw_path;
    end
end