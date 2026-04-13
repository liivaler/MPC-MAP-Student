function public_vars = student_workspace(read_only_vars, public_vars)

    % jednoduchý pohyb robota, aby lokalizace měla z čeho poznávat polohu
    public_vars.motion_vector = [0.3, 0.25];

    % inicializace částic
    if ~isfield(public_vars, 'pf_initialized')
        public_vars.pf_initialized = true;

        N = 300;

        xmin = read_only_vars.map.limits(1);
        xmax = read_only_vars.map.limits(3);
        ymin = read_only_vars.map.limits(2);
        ymax = read_only_vars.map.limits(4);

        particles = zeros(N,3);
        count = 0;

        while count < N
            x = xmin + (xmax-xmin)*rand;
            y = ymin + (ymax-ymin)*rand;
            theta = 2*pi*rand;

            inside = false;
            for k = 1:size(read_only_vars.map.gnss_denied,1)
                xv = read_only_vars.map.gnss_denied(k,1:2:end);
                yv = read_only_vars.map.gnss_denied(k,2:2:end);
                if inpolygon(x, y, xv, yv)
                    inside = true;
                end
            end

            if inside
                count = count + 1;
                particles(count,:) = [x, y, theta];
            end
        end

        public_vars.particles = particles;
    end
    % if mod(read_only_vars.counter,10) == 0
    %     disp(size(public_vars.particles))
    %     disp(public_vars.particles(1:5,:))
    % end
    % if mod(read_only_vars.counter,10) == 0
    %     disp(mean(public_vars.particles(:,1:2),1))
    % end
    % update PF
    [public_vars.particles, weights] = update_particle_filter(read_only_vars, public_vars);

    [~, idx] = sort(weights, 'descend');
    best = public_vars.particles(idx(1:30), :);
    
    public_vars.estimated_pose = mean(best, 1);
    public_vars.estimated_pose(3) = atan2(mean(sin(best(:,3))), mean(cos(best(:,3))));
end