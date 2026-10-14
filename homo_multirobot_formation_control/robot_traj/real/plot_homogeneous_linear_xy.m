% Compare raw map-frame XY trajectories from one homogeneous and one linear trial.
% Edit only these two directory names to select another experiment pair.
homogeneous_trial_dir = '6d_artstein_no_forw_vs_forw_20260922_112130';
linear_trial_dir = '6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051';

script_dir = fileparts(mfilename('fullpath'));
homogeneous = read_trial(script_dir, homogeneous_trial_dir, '齐次控制');
linear = read_trial(script_dir, linear_trial_dir, '线性控制');

fig = figure('Color', 'w', 'Position', [100, 100, 760, 560]);
hold on;

homogeneous_color = [0.00, 0.45, 0.74];
linear_color = [0.85, 0.33, 0.10];

plot(homogeneous.leader_x_m, homogeneous.leader_y_m, '-', ...
    'Color', homogeneous_color, 'LineWidth', 1.5);
plot(homogeneous.follower_x_m, homogeneous.follower_y_m, '--', ...
    'Color', homogeneous_color, 'LineWidth', 1.5);
plot(linear.leader_x_m, linear.leader_y_m, '-', ...
    'Color', linear_color, 'LineWidth', 1.5);
plot(linear.follower_x_m, linear.follower_y_m, '--', ...
    'Color', linear_color, 'LineWidth', 1.5);

axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
legend({'齐次 Leader', '齐次 Follower', '线性 Leader', '线性 Follower'}, ...
    'Location', 'best');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'LineWidth', 1);

exportgraphics(fig, fullfile(script_dir, 'homogeneous_vs_linear_xy.png'), ...
    'Resolution', 300);
exportgraphics(fig, fullfile(script_dir, 'homogeneous_vs_linear_xy.pdf'), ...
    'ContentType', 'vector');


function trial = read_trial(script_dir, trial_dir, group_name)
csv_path = fullfile(script_dir, trial_dir, 'raw.csv');
if ~isfile(csv_path)
    error('plot_homogeneous_linear_xy:MissingCsv', ...
        '%s组找不到文件: %s', group_name, csv_path);
end

trial = readtable(csv_path);
required_columns = {'leader_x_m', 'leader_y_m', ...
    'follower_x_m', 'follower_y_m'};
missing_columns = required_columns(~ismember(required_columns, ...
    trial.Properties.VariableNames));
if ~isempty(missing_columns)
    error('plot_homogeneous_linear_xy:MissingColumn', ...
        '%s组的 raw.csv 缺少列: %s', group_name, ...
        strjoin(missing_columns, ', '));
end
end
