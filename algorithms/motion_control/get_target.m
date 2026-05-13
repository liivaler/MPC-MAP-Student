function [target] = get_target(estimated_pose, path)
% GET_TARGET
% Pomocná funkce pro výběr bodu na trase před robotem.
% Ve finální verzi se hlavní výběr cíle řeší přímo v plan_motion.m.

    if isempty(path)
        target = [0, 0];
        return;
    end

    x = estimated_pose(1);
    y = estimated_pose(2);

    d2 = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    [~, nearest_idx] = min(d2);

    % malý lookahead, aby nepřeskakoval rohy v indoor mapě
    lookahead = 1;

    target_idx = min(nearest_idx + lookahead, size(path,1));
    target = path(target_idx,:);
end