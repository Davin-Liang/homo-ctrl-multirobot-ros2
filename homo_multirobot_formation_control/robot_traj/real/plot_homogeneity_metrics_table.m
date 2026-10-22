% 汇总不同齐次度无前馈实物试验的六项编队指标，并导出论文表格图片。
ideal_radius_m = 1.0;
steady_window_s = 10.0;
script_dir = fileparts(mfilename('fullpath'));

nu_values = [-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0];
trial_groups = {
    {'6d_artstein_no_forw_vs_forw_20260922_112130', '6d_artstein_no_forw_vs_forw_20260922_112637', '6d_artstein_no_forw_vs_forw_20260922_113126'};
    {'6d_artstein_nu-0.9(no_fora)_20260922_161938', '6d_artstein_nu-0.9(no_fora)_20260922_162215', '6d_artstein_nu-0.9(no_fora)_20260922_162535'};
    {'6d_artstein_nu-0.8(no_fora)_20260922_162911', '6d_artstein_nu-0.8(no_fora)_20260922_163147', '6d_artstein_nu-0.8(no_fora)_20260922_163348'};
    {'6d_artstein_nu-0.7(no_fora)_20260922_163733', '6d_artstein_nu-0.7(no_fora)_20260922_164127', '6d_artstein_nu-0.7(no_fora)_20260922_164349'};
    {'6d_artstein_nu-0.6(no_fora)_20260922_165110', '6d_artstein_nu-0.6(no_fora)_20260922_165453', '6d_artstein_nu-0.6(no_fora)_20260922_165759'};
    {'6d_artstein_nu-0.5(no_fora)_20260922_170125', '6d_artstein_nu-0.5(no_fora)_20260922_170509', '6d_artstein_nu-0.5(no_fora)_20260922_170816'};
    {'6d_artstein_no_forward_nu-0.4_20260928_171712', '6d_artstein_no_forward_nu-0.4_20260928_171938', '6d_artstein_no_forward_nu-0.4_20260928_172243'};
    {'6d_artstein_no_forward_nu-0.3_20260928_172716', '6d_artstein_no_forward_nu-0.3_20260928_172953', '6d_artstein_no_forward_nu-0.3_20260928_173238'};
    {'6d_artstein_no_forward_nu-0.2_20260928_201610', '6d_artstein_no_forward_nu-0.2_20260928_201956', '6d_artstein_no_forward_nu-0.2_20260928_202222'};
    {'6d_artstein_no_forward_nu-0.1_20260928_202437', '6d_artstein_no_forward_nu-0.1_20260928_202638', '6d_artstein_no_forward_nu-0.1_20260928_202854'};
    {'6d_artstein_no_forward_nu-0_20260928_203052', '6d_artstein_no_forward_nu-0_20260928_203325', '6d_artstein_no_forward_nu-0_20260928_203536'};
};
homogeneity_groups = repmat(struct('nu', 0.0, 'trials', {{}}), ...
    1, numel(nu_values));
for group_index = 1:numel(nu_values)
    homogeneity_groups(group_index).nu = nu_values(group_index);
    homogeneity_groups(group_index).trials = trial_groups{group_index};
end

metric_names = {'full_distance_mae', 'steady_distance_mae', ...
    'full_yaw_rms', 'steady_yaw_rms', ...
    'full_velocity_mae', 'steady_velocity_mae'};
column_names = {'Full Distance MAE (m)', 'Steady Distance MAE (m)', ...
    'Full Yaw RMS (rad)', 'Steady Yaw RMS (rad)', ...
    'Full Velocity MAE (m/s)', 'Steady Velocity MAE (m/s)'};

group_data = cell(numel(homogeneity_groups), numel(metric_names));
row_names = cell(numel(homogeneity_groups), 1);
summary_values = strings(numel(homogeneity_groups), numel(metric_names));
for group_index = 1:numel(homogeneity_groups)
    group = homogeneity_groups(group_index);
    if numel(group.trials) ~= 3
        error('plot_homogeneity_metrics_table:TrialCount', ...
            'nu=%.1f 必须配置三次试验。', group.nu);
    end

    values = zeros(3, numel(metric_names));
    for trial_index = 1:3
        values(trial_index, :) = read_trial_metrics(script_dir, ...
            group.trials{trial_index}, group.nu, ideal_radius_m, steady_window_s);
    end
    means = mean(values, 1);
    deviations = std(values, 0, 1);
    for metric_index = 1:numel(metric_names)
        group_data{group_index, metric_index} = ...
            sprintf('%.4f +/- %.4f', means(metric_index), deviations(metric_index));
        summary_values(group_index, metric_index) = group_data{group_index, metric_index};
    end
    row_names{group_index} = sprintf('nu = %.1f', group.nu);
end

summary_table = array2table(summary_values, 'VariableNames', metric_names, ...
    'RowNames', row_names);
disp(summary_table);

fig = figure('Color', 'w', 'Position', [100, 100, 1600, 540]);
uitable(fig, 'Data', group_data, 'ColumnName', column_names, ...
    'RowName', row_names, 'Units', 'normalized', ...
    'Position', [0, 0, 1, 1], 'FontName', 'Times New Roman', ...
    'FontSize', 11, 'ColumnWidth', {200, 210, 185, 200, 205, 220});

exportgraphics(fig, fullfile(script_dir, 'homogeneity_metrics_table.png'), ...
    'Resolution', 300);
exportgraphics(fig, fullfile(script_dir, 'homogeneity_metrics_table.pdf'), ...
    'ContentType', 'vector');


function metrics = read_trial_metrics(script_dir, trial_dir, group_nu, ...
    ideal_radius_m, steady_window_s)
csv_path = fullfile(script_dir, trial_dir, 'raw.csv');
if ~isfile(csv_path)
    error('plot_homogeneity_metrics_table:MissingCsv', ...
        'nu=%.1f 找不到文件: %s', group_nu, csv_path);
end

trial = readtable(csv_path);
required_columns = {'time_s', 'distance_m', ...
    'leader_vx_map_ms', 'leader_vy_map_ms', ...
    'follower_vx_map_ms', 'follower_vy_map_ms', ...
    'leader_yaw_rad', 'follower_yaw_rad'};
missing_columns = required_columns(~ismember(required_columns, ...
    trial.Properties.VariableNames));
if ~isempty(missing_columns)
    error('plot_homogeneity_metrics_table:MissingColumn', ...
        'nu=%.1f 文件 %s 缺少列: %s', group_nu, csv_path, ...
        strjoin(missing_columns, ', '));
end

distance_error = abs(trial.distance_m - ideal_radius_m);
yaw_error = wrap_to_pi(trial.follower_yaw_rad - trial.leader_yaw_rad);
velocity_error = hypot( ...
    trial.follower_vx_map_ms - trial.leader_vx_map_ms, ...
    trial.follower_vy_map_ms - trial.leader_vy_map_ms);
steady_mask = trial.time_s >= trial.time_s(end) - steady_window_s;
if ~any(steady_mask)
    steady_mask = true(height(trial), 1);
end

metrics = [mean(distance_error), mean(distance_error(steady_mask)), ...
    sqrt(mean(yaw_error .^ 2)), sqrt(mean(yaw_error(steady_mask) .^ 2)), ...
    mean(velocity_error), mean(velocity_error(steady_mask))];
end


function wrapped = wrap_to_pi(angle)
wrapped = atan2(sin(angle), cos(angle));
end
