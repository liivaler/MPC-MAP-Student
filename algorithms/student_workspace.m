function public_vars = student_workspace(read_only_vars, public_vars)

    % MoCap jako odhad polohy
    public_vars.estimated_pose = read_only_vars.mocap_pose;

    % zavolání řízení pohybu
    [motion_vector, public_vars] = plan_motion(read_only_vars, public_vars);

    % uložení rychlostí robota
    public_vars.motion_vector = motion_vector;

end