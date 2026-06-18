%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4D 连续边界投影 + 切向修正编队控制 (fixed world-frame angle)
%%
%%   d_des = radius * [cos(theta_d); sin(theta_d)]   -- 圆上目标点(固定角度)
%%   e_pos = dpos - d_des                             -- 含径向+切向分量
%%
%% 关键设计: theta_d 固定在 world frame 的固定角度, 不跟踪速度方向。
%%   速度方向在圆周运动中以 1 rad/s 旋转, 跟随者追不上移动靶导致振荡。
%%   固定角度等价于离散多边形 m_p=1 (只有一个编队点, 无切换)。
%%
%% 对比:
%%   - lpc_hpc_distance_square: 离散多边形 (m_p=4, tol 切换)
%%   - demo_4d_cont:            连续边界投影 (纯径向, 无角度约束)
%%   - demo_4d_cont_tangent:    连续边界投影 + 固定角度切向约束 (本文件)
%%
%% 运行条件与 lpc_hpc_distance_square.m 一致:
%%   m=2, radius=1, h=0.01, Tmax=30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%% ====== 运行条件 ======
t = 0; Tmax = 30;
h = 0.01;
m = 2;

A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)*1/m];

x1 = [1; 0; 0; 0];
x2 = [5; 1; 0; 0];

radius = 1;
theta_d = pi;           % 固定目标方位角 (world frame): pi = 在领航者-x方向

%% ====== 控制参数 ======
gain_min  = 4;
gain_max  = 40;
eps_blend = 0.15;

%% ====== 测量噪声 (模拟 ROS EKF+TF 链路) ======
noise_pos_std = 0.0;   % 位置噪声标准差 (m)   -- TF + EKF pose
noise_vel_std = 0.0;    % 速度噪声标准差 (m/s)  -- EKF twist, 比位置更噪
rng(42);                 % 固定随机种子, 可复现

%% ====== 初始误差 ======
dpos = [x2(1)-x1(1); x2(2)-x1(2)];
d_des = radius * [cos(theta_d); sin(theta_d)];
e_pos = dpos - d_des;
e = [e_pos; x2(3)-x1(3); x2(4)-x1(4)];

%% ====== 初始增益 ======
[~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
[~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);

lambda = diag([a b]);
k2 = -2 * lambda;
k1 = lambda * (k2 + lambda) / m;
k_lin = [k1 k2];

fprintf('=== 初始状态 ===\n');
fprintf('x1 = [%.4f; %.4f; %.4f; %.4f]\n', x1);
fprintf('x2 = [%.4f; %.4f; %.4f; %.4f]\n', x2);
fprintf('dpos = [%.4f; %.4f], r_dist = %.4f\n', dpos(1), dpos(2), norm(dpos));
fprintf('theta_d = %.4f (fixed world-frame)\n', theta_d);
fprintf('d_des = [%.4f; %.4f]\n', d_des);
fprintf('e = [%.4f; %.4f; %.4f; %.4f]\n', e);
fprintf('a=%.4f b=%.4f\n', a, b);

%% ====== LPC -> HPC ======
[K0, G0, P, nu_min, nu_max] = lpc2hpc(A, B, k_lin);
nu = nu_min;
Gd = eye(4) + nu * G0;

fprintf('nu = %.6f,  u2_init = [%.4f; %.4f]\n\n', nu, k_lin * e);

%% ====== 日志 ======
tl = []; xl1 = []; ul1 = [];
xl2 = []; ul2 = [];
el = []; ul = [];
al = []; bl = [];

print_step = 500;
step = 0;

while t < Tmax
    step = step + 1;

    %% --- 领航者动力学 (clean) ---
    u1 = -[eye(2) eye(2)] * x1 + 1 * [sin(t); cos(t)];
    x1 = x1 + h * (A * x1 + B * u1);

    %% --- 构造带噪测量 (模拟 ROS EKF+TF 链路) ---
    %   控制器看到的 = 真值 + 测量噪声
    %   leader 和 follower 状态都来自 EKF, 各自独立加噪
    x1_meas = x1 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];
    x2_meas = x2 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];

    %% --- 误差 (基于带噪测量) ---
    dpos_meas = [x2_meas(1)-x1_meas(1); x2_meas(2)-x1_meas(2)];
    d_des = radius * [cos(theta_d); sin(theta_d)];
    e_pos = dpos_meas - d_des;
    e = [e_pos; x2_meas(3)-x1_meas(3); x2_meas(4)-x1_meas(4)];

    %% --- 自适应增益 (平滑混合) ---
    [~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
    [~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);

    lambda = diag([a b]);
    k2 = -2 * lambda;
    k1 = lambda * (k2 + lambda) / m;
    k_lin = [k1 k2];

    %% --- 重算 HPC ---
    [K0, G0, P, nu_min, nu_max] = lpc2hpc(A, B, k_lin);
    nu = nu_min;
    Gd = eye(4) + nu * G0;

    %% --- HPC 控制律 (基于带噪误差) ---
    nx = hnorm(e, Gd, P);
    c = max(min(1, nx), 0.1);
    u2 = c^(1 + nu) * k_lin * expm(Gd * (1 - log(c))) * e;

    %% --- 跟随者动力学 (clean, 真值积分) ---
    x2 = x2 + h * (A * x2 + B * u2);

    %% --- 日志 (记录 clean 真值用于绘图和评估) ---
    dpos_true = [x2(1)-x1(1); x2(2)-x1(2)];
    e_true = [dpos_true - d_des; x2(3)-x1(3); x2(4)-x1(4)];
    t = t + h;
    tl = [tl t];
    xl1 = [xl1 x1];
    ul1 = [ul1 u1];
    xl2 = [xl2 x2];
    ul2 = [ul2 u2];
    el = [el e_true];        % 绘图用真值误差
    ul = [ul u2];
    al = [al a];
    bl = [bl b];

    if mod(step, print_step) == 0
        r_true = norm(dpos_true);
        fprintf('t=%.2f | r_true=%.3f | err_pos(meas)=[%+.4f %+.4f] | ', ...
            t, r_true, e(1), e(2));
        fprintf('a=%.2f b=%.2f | u2=[%+.4f %+.4f]\n', a, b, u2(1), u2(2));
    end
end

fprintf('\n=== 最终状态 ===\n');
dpos_final = [x2(1)-x1(1); x2(2)-x1(2)];
r_final = norm(dpos_final);
angle_final = atan2(dpos_final(2), dpos_final(1));
fprintf('x2 = [%.4f; %.4f; %.4f; %.4f]\n', x2);
fprintf('r_dist = %.4f (deviation = %.4f)  angle_err = %.4f rad\n', ...
    r_final, r_final - radius, ...
    atan2(sin(angle_final-theta_d), cos(angle_final-theta_d)));

%% ========================================================================
E = sqrt(el(1,:).^2 + el(2,:).^2);

% ---- Figure 1: X axis ----
figure(1); hold on;
plot(tl, xl1(1,:), 'r', 'LineWidth', 2);
plot(tl, xl2(1,:), 'b', 'LineWidth', 2);
xlim([0 Tmax])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$x$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$x_1$', '$x_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 2: Y axis ----
figure(2); hold on;
plot(tl, xl1(2,:), 'r', 'LineWidth', 2);
plot(tl, xl2(2,:), 'b', 'LineWidth', 2);
xlim([0 Tmax]); ylim([-1.5 1.5])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$y$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$y_1$', '$y_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 3: XY trajectory ----
figure(3); hold on;
plot(xl1(1,:), xl1(2,:), 'r', 'LineWidth', 2);
plot(xl2(1,:), xl2(2,:), 'b', 'LineWidth', 2);
xlim([-1.5 2.5]); ylim([-1.5 1.5])
xlabel('$x$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$y$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$r_1$', '$r_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 4: Control input ----
figure(4); hold on;
plot(tl, ul2(1,:), 'r', 'LineWidth', 2);
plot(tl, ul2(2,:), 'b', 'LineWidth', 2);
ylim([-15 10])
xlabel('$t$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$u$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$u_x$', '$u_y$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 5: Position error components ----
figure(5); hold on;
plot(tl, el(1,:), 'r', 'LineWidth', 2);
plot(tl, el(2,:), 'b', 'LineWidth', 2);
xlabel('$t$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$e_{pos}$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$e_x$', '$e_y$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 6: Distance to boundary ----
dist_bnd = sqrt((xl2(1,:)-xl1(1,:)).^2 + (xl2(2,:)-xl1(2,:)).^2) - radius;
figure(6); hold on;
plot(tl, dist_bnd, 'b', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('dist to boundary (m)', 'FontSize', 20, 'Interpreter', 'latex')
legend('$|d_{pos}| - r$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 7: Position error norm ----
figure(7); hold on;
plot(tl, E, 'r', 'LineWidth', 2);
ylim([0 3.5])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$||e_{pos}||$', 'FontSize', 20, 'Interpreter', 'latex')
grid on;

% ---- Figure 8: Gain history ----
figure(8); hold on;
plot(tl, al, 'r', 'LineWidth', 1.5, 'DisplayName', '$a$ (x)');
plot(tl, bl, 'b', 'LineWidth', 1.5, 'DisplayName', '$b$ (y)');
yline(gain_min, 'k--', 'LineWidth', 1);
yline(gain_max, 'k:', 'LineWidth', 1);
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('Gain', 'FontSize', 20, 'Interpreter', 'latex')
legend('show', 'FontSize', 16, 'Box', 'off', 'Interpreter', 'latex');
grid on;

% ---- Figure 9: Angle tracking ----
actual_angles = atan2(xl2(2,:)-xl1(2,:), xl2(1,:)-xl1(1,:));
angle_errors = atan2(sin(actual_angles - theta_d), cos(actual_angles - theta_d));
figure(9); hold on;
yyaxis left;
plot(tl, actual_angles, 'b', 'LineWidth', 1.5);
yline(theta_d, 'k--', 'LineWidth', 1);
ylabel('Angle (rad)', 'FontSize', 16, 'Interpreter', 'latex');
yyaxis right;
plot(tl, angle_errors, 'r', 'LineWidth', 1.5);
ylabel('Angle error (rad)', 'FontSize', 16, 'Interpreter', 'latex');
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex');
title('Angle Tracking on Safety Circle', 'FontSize', 14);
legend('$\theta$ actual', '$\theta_d$', '$\Delta\theta$', ...
    'FontSize', 14, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% ====== Y 轴跟踪总结 ======
fprintf('\n===== Y-axis Tracking Summary (fixed-angle tangential) =====\n');
y_err = xl2(2,:) - xl1(2,:);
fprintf('  RMS  |Y| error:  %.4f\n', sqrt(mean(y_err.^2)));
fprintf('  Max  |Y| error:  %.4f\n', max(abs(y_err)));
fprintf('  Mean Y   error:  %.4f\n', mean(y_err));
check_times = [5, 10, 15, 20, 25, 30];
fprintf('  Sampled Y values:\n');
for ti = check_times
    [~, idx] = min(abs(tl - ti));
    fprintf('    t=%-2.0f  |  y_leader= % .4f  y_follower= % .4f  err= % .4f\n', ...
        tl(idx), xl1(2,idx), xl2(2,idx), y_err(idx));
end
fprintf('  Final follower Y: %.4f\n', xl2(2,end));

disp('Done!');

%% ========================================================================
function [val_ratio, a] = smooth_gain(m, e_pos, e_vel, gain_min, gain_max, eps_blend)
    if abs(e_pos) > 1e-6
        val_ratio = -m * e_vel / e_pos;
    else
        val_ratio = 0;
    end
    val_ratio = max(min(val_ratio, gain_max), -gain_max);
    a_adaptive = max(val_ratio, gain_min);
    w = min(abs(e_pos) / eps_blend, 1.0);
    a = w * a_adaptive + (1 - w) * gain_min;
end
