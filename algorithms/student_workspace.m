function [public_vars] = student_workspace(read_only_vars, public_vars)
% STUDENT_WORKSPACE
% Modes:
%   'lidar' - Task 2-4 for LiDAR
%   'gnss'  - Task 2-4 for GNSS
%   'task5'  - Task 5
mode = 'task5';   % <<< PREPINAT 

persistent lidar_data gnss_data lidar_done gnss_done

if isempty(lidar_data)
    lidar_data = [];
end

if isempty(gnss_data)
    gnss_data = [];
end

if isempty(lidar_done)
    lidar_done = false;
end

if isempty(gnss_done)
    gnss_done = false;
end

% init puvodni pipeline
if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

switch mode

    % ==================================================
    % MODE: LIDAR
    % ==================================================
    case 'lidar'

        lidar = read_only_vars.lidar_distances(:).';

        if numel(lidar) == 8
            lidar_row = lidar;
            lidar_row(~isfinite(lidar_row)) = NaN;

            if sum(isfinite(lidar_row)) >= 6
                lidar_data = [lidar_data; lidar_row];
            end
        end

        lidar_count = size(lidar_data,1);

        if mod(read_only_vars.counter,20) == 0
            fprintf('Step %4d | LiDAR stored rows: %3d\n', ...
                read_only_vars.counter, lidar_count);
        end

        if lidar_count >= 100 && ~lidar_done
            sigma_lidar = std(lidar_data, 0, 1, 'omitnan');
            cov_lidar   = cov(lidar_data, 'partialrows');

            fprintf('\n=== LiDAR DONE ===\n');
            disp('Sigma LiDAR:');
            disp(sigma_lidar);
            disp('Covariance LiDAR:');
            disp(cov_lidar);

            figure;
            subplot(2,1,1);
            histogram(lidar_data(isfinite(lidar_data(:,1)),1));
            title('LiDAR Histogram - Channel 1');
            xlabel('Distance');
            ylabel('Count');

            subplot(2,1,2);
            histogram(lidar_data(isfinite(lidar_data(:,2)),2));
            title('LiDAR Histogram - Channel 2');
            xlabel('Distance');
            ylabel('Count');

            sig1 = sigma_lidar(1);
            if isfinite(sig1) && sig1 > 0
                x1 = linspace(-4*sig1, 4*sig1, 300);
                y1 = norm_pdf(x1, 0, sig1);

                figure;
                plot(x1, y1, 'LineWidth', 2);
                grid on;
                title('Normal PDF - LiDAR Channel 1');
                xlabel('x');
                ylabel('pdf');
            end

            public_vars.week2.lidar_data  = lidar_data;
            public_vars.week2.sigma_lidar = sigma_lidar;
            public_vars.week2.cov_lidar   = cov_lidar;

            lidar_done = true;

            fprintf('LiDAR plots generated. Pausing for 8 seconds...\n');
            drawnow;
            pause(8);
        end

        public_vars.motion_vector = [0, 0];

    % ==================================================
    % MODE: GNSS
    % ==================================================
    case 'gnss'

        gnss = read_only_vars.gnss_position(:).';

        if numel(gnss) == 2
            gnss_data = [gnss_data; gnss];
        end

        gnss_valid_rows = gnss_data(all(isfinite(gnss_data),2), :);
        gnss_count = size(gnss_valid_rows,1);

        if mod(read_only_vars.counter,20) == 0
            fprintf('Step %4d | GNSS stored = %3d | valid = %3d\n', ...
                read_only_vars.counter, size(gnss_data,1), gnss_count);
        end

        if gnss_count >= 100 && ~gnss_done
            sigma_gnss = std(gnss_valid_rows, 0, 1);
            cov_gnss   = cov(gnss_valid_rows);

            fprintf('\n=== GNSS DONE ===\n');
            disp('Sigma GNSS:');
            disp(sigma_gnss);
            disp('Covariance GNSS:');
            disp(cov_gnss);

            figure;
            subplot(2,1,1);
            histogram(gnss_valid_rows(:,1));
            title('GNSS Histogram - X');
            xlabel('Position X');
            ylabel('Count');

            subplot(2,1,2);
            histogram(gnss_valid_rows(:,2));
            title('GNSS Histogram - Y');
            xlabel('Position Y');
            ylabel('Count');

            sigx = sigma_gnss(1);
            if isfinite(sigx) && sigx > 0
                x2 = linspace(-4*sigx, 4*sigx, 300);
                y2 = norm_pdf(x2, 0, sigx);

                figure;
                plot(x2, y2, 'LineWidth', 2);
                grid on;
                title('Normal PDF - GNSS X');
                xlabel('x');
                ylabel('pdf');
            end

            public_vars.week2.gnss_data  = gnss_valid_rows;
            public_vars.week2.sigma_gnss = sigma_gnss;
            public_vars.week2.cov_gnss   = cov_gnss;

            gnss_done = true;

            fprintf('GNSS plots generated. Pausing for 8 seconds...\n');
            drawnow;
            pause(8);
        end

        public_vars.motion_vector = [0, 0];

    % ==================================================
    % MODE: TASK 5
    % ==================================================
    case 'task5'

    if (read_only_vars.counter == 1)
        public_vars.estimated_pose = [];
        public_vars.path = [];
        public_vars.particles = [];
    end

    public_vars = plan_motion(read_only_vars, public_vars);

    otherwise
        error('Unknown mode. Use ''lidar'', ''gnss'', or ''task5''.');
end

% puvodni pipeline
if ~strcmp(mode, 'task5')
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    public_vars.estimated_pose = estimate_pose(public_vars);
    public_vars.path = plan_path(read_only_vars, public_vars);
end

end
