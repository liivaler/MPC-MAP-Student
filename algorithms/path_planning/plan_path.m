function [path] = plan_path(read_only_vars, public_vars)
% PLAN_PATH - Week 6

    raw_path = astar(read_only_vars, public_vars);

    if isempty(raw_path)
        path = [];
        return;
    end

    path = smooth_path(raw_path, read_only_vars);

end

% function [path] = plan_path(read_only_vars, public_vars)
% % PLAN_PATH - Week 6
% 
%     disp('PLANNING WITHOUT SMOOTHING');
%     path = astar(read_only_vars, public_vars);
% 
% end