%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4D 连续边界投影编队控制 (Continuous Boundary Projection)
%%
%% 运行条件与 lpc_hpc_distance_square.m 一致:
%%   m=2, radius=1, h=0.01, Tmax=30, 相同初始状态和领航者轨迹
%%
%% 唯一区别: 编队策略为连续边界投影 (径向投影到安全圆)
%%   - 无 m_p、tol、dl 离散编队点, 无切换逻辑
%%   - 每步重算 k_lin 和 HPC 参数
%%   - 增益计算加入防除零保护和上界钳位
%%      (原因: 投影误差在接近安全圆时 e_pos → 0, 导致 ev/ep 爆炸)
%%
%% 对照 C++: homo_controller_4d_cont.hpp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%% ====== 运行条件 (与 lpc_hpc_distance_square.m 一致) ======
t = 0; Tmax = 30;
h = 0.01;           % 采样周期
m = 2;              % 质量

A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)*1/m];

x1 = [1; 0; 0; 0];          % 领航者
x2 = [5; 1; 0; 0];          % 跟随者

radius = 1;         % 安全圆半径

%% ====== 初始误差 (连续边界投影) ======
dpx = x2(1) - x1(1);
dpy = x2(2) - x1(2);
r_dist = max(sqrt(dpx^2 + dpy^2), 1e-3);
dx = radius * dpx / r_dist;
dy = radius * dpy / r_dist;
e = [dpx - dx; dpy - dy; x2(3) - x1(3); x2(4) - x1(4)];

%% ====== 控制参数 ======
gain_min  = 4;      % 基础增益下界 (原版切换时用4, 对应 λ=-a/m=-2)
gain_max  = 40;     % 增益上界, 防止 ev/ep 爆炸
eps_blend = 0.15;   % 位置误差小于此值时平滑退化为固定增益

%% ====== 初始线性增益 (平滑混合, 消除锯齿) ======
[~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
[~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);

lambda = diag([a b]);
k2 = -2 * lambda;
k1 = lambda * (k2 + lambda) / m;
k_lin = [k1 k2];

fprintf('=== 初始状态 ===\n');
fprintf('x1 = [%.4f; %.4f; %.4f; %.4f]\n', x1);
fprintf('x2 = [%.4f; %.4f; %.4f; %.4f]\n', x2);
fprintf('dpos = [%.4f; %.4f], r_dist = %.4f\n', dpx, dpy, r_dist);
fprintf('projection = [%.4f; %.4f]\n', dx, dy);
fprintf('e = [%.4f; %.4f; %.4f; %.4f]\n', e);
fprintf('a=%.4f b=%.4f\n', a, b);
fprintf('k_lin = [%.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f]\n', k_lin');

%% ====== LPC -> HPC 升级 ======
[K0, G0, P, nu_min, nu_max] = lpc2hpc(A, B, k_lin);
nu = nu_min;
Gd = eye(4) + nu * G0;

fprintf('nu = %.6f\n', nu);
fprintf('initial u2 = k_lin*e = [%.4f; %.4f]\n\n', k_lin * e);

%% ====== 日志 ======
tl = []; xl1 = []; ul1 = [];
xl2 = []; ul2 = [];
el = []; ul = [];
al = []; bl = [];

print_step = 500;  % 每隔多少步打印一次
step = 0;

while t < Tmax
    step = step + 1;

    %% --- 领航者 (与原版一致) ---
    u1 = -[eye(2) eye(2)] * x1 + 1 * [sin(t); cos(t)];
    x1 = x1 + h * (A * x1 + B * u1);

    %% --- 连续边界投影误差 ---
    dpx = x2(1) - x1(1);
    dpy = x2(2) - x1(2);
    r_dist = max(sqrt(dpx^2 + dpy^2), 1e-3);
    dx = radius * dpx / r_dist;
    dy = radius * dpy / r_dist;
    e = [dpx - dx; dpy - dy; x2(3) - x1(3); x2(4) - x1(4)];

    %% --- 自适应线性增益 (平滑混合, 消除锯齿) ---
    [~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
    [~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);

    lambda = diag([a b]);
    k2 = -2 * lambda;
    k1 = lambda * (k2 + lambda) / m;
    k_lin = [k1 k2];

    %% --- 重算 HPC 参数 ---
    [K0, G0, P, nu_min, nu_max] = lpc2hpc(A, B, k_lin);
    nu = nu_min;
    Gd = eye(4) + nu * G0;

    %% --- HPC 控制律 ---
    nx = hnorm(e, Gd, P);
    c = max(min(1, nx), 0.1);
    u2 = c^(1 + nu) * k_lin * expm(Gd * (1 - log(c))) * e;

    %% --- 跟随者动力学 ---
    x2 = x2 + h * (A * x2 + B * u2);

    %% --- 日志 ---
    t = t + h;
    tl = [tl t];
    xl1 = [xl1 x1];
    ul1 = [ul1 u1];
    xl2 = [xl2 x2];
    ul2 = [ul2 u2];
    el = [el e];
    ul = [ul u2];
    al = [al a];
    bl = [bl b];

    %% --- 定期打印 ---
    if mod(step, print_step) == 0
        fprintf('t=%.2f | r_dist=%.3f | err_pos=[%.4f %.4f] | ', ...
            t, r_dist, e(1), e(2));
        fprintf('a=%.2f b=%.2f | ', a, b);
        fprintf('nx=%.4f c=%.4f | u2=[%.4f %.4f]\n', ...
            nx, c, u2(1), u2(2));
    end
end

fprintf('\n=== 最终状态 ===\n');
fprintf('t = %.2f\n', t);
fprintf('x2 = [%.4f; %.4f; %.4f; %.4f]\n', x2);
fprintf('r_dist = %.4f (偏差 = %.4f)\n', r_dist, r_dist - radius);

%% ====== 复合位置误差范数 ======
E = sqrt(el(1,:).^2 + el(2,:).^2);

%% ========================================================================
%%  绘图 (与原版图表对应)
%% ========================================================================

%% Figure 1: X 轴
figure(1); hold on;
plot(tl, xl1(1,:), 'r', 'LineWidth', 2);
plot(tl, xl2(1,:), 'b', 'LineWidth', 2);
xlim([0 Tmax])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$x$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$x_1$', '$x_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 2: Y 轴  <--- 重点观察
figure(2); hold on;
plot(tl, xl1(2,:), 'r', 'LineWidth', 2);
plot(tl, xl2(2,:), 'b', 'LineWidth', 2);
xlim([0 Tmax])
ylim([-0.8 1.2])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$y$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$y_1$', '$y_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 3: XY 轨迹
figure(3); hold on;
plot(xl1(1,:), xl1(2,:), 'r', 'LineWidth', 2);
plot(xl2(1,:), xl2(2,:), 'b', 'LineWidth', 2);
ylim([-0.8 1.2])
xlabel('$x$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$y$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$r_1$', '$r_2$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
xlim([-1.5 2.5]); grid on;

%% Figure 4: 控制量
figure(4); hold on;
plot(tl, ul2(1,:), 'r', 'LineWidth', 2);
plot(tl, ul2(2,:), 'b', 'LineWidth', 2);
ylim([-12 6])
xlabel('$t$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$u$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$u_x$', '$u_y$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 5: 位置误差分量
figure(5); hold on;
plot(tl, el(1,:), 'r', 'LineWidth', 2);
plot(tl, el(2,:), 'b', 'LineWidth', 2);
xlabel('$t$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$\ell$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$\ell_x$', '$\ell_y$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 6: 跟随者到安全边界的距离 (>0 圆外, <0 圆内)
dist_to_boundary = sqrt((xl2(1,:)-xl1(1,:)).^2 + (xl2(2,:)-xl1(2,:)).^2) - radius;
figure(6); hold on;
plot(tl, dist_to_boundary, 'b', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('dist to boundary (m)', 'FontSize', 20, 'Interpreter', 'latex')
legend('$|d_{pos}| - r$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 7: 复合位置误差范数 (对应原版 Figure 7)
figure(7); hold on;
plot(tl, E, 'r', 'LineWidth', 2);
ylim([0 3.5])
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('$e$', 'FontSize', 20, 'Interpreter', 'latex')
legend('$e_{hom}$', 'FontSize', 20, 'Box', 'off', 'Interpreter', 'latex');
grid on;

%% Figure 8: 增益时间历程 (a, b) -- 验证平滑混合
figure(8); hold on;
plot(tl, al, 'r', 'LineWidth', 1.5, 'DisplayName', '$a$ (x-channel)');
plot(tl, bl, 'b', 'LineWidth', 1.5, 'DisplayName', '$b$ (y-channel)');
yline(gain_min, 'k--', 'LineWidth', 1);
yline(gain_max, 'k:', 'LineWidth', 1);
xlabel('$t(s)$', 'FontSize', 20, 'Interpreter', 'latex')
ylabel('Gain', 'FontSize', 20, 'Interpreter', 'latex')
title('Adaptive Gain History (Smooth Blend)', 'FontSize', 14);
legend('show', 'FontSize', 16, 'Box', 'off', 'Interpreter', 'latex');
grid on;

fprintf('\n===== Y-axis Tracking Summary (continuous projection) =====\n');
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
%%  辅助函数: 平滑混合增益
%%
%%  w = min(|e_pos| / eps_blend, 1)           -- 混合权重 (0→1)
%%  val_ratio  = clamp(-m * e_vel / e_pos, ±gain_max)
%%  a_adaptive = max(val_ratio, gain_min)
%%  a = w * a_adaptive + (1-w) * gain_min     -- 平滑过渡
%%
%%  e_pos 远离零时: w≈1 → 完全自适应
%%  e_pos 接近零时: w≈0 → 固定最小增益 (消除锯齿)
%% ========================================================================
function [val_ratio, a] = smooth_gain(m, e_pos, e_vel, gain_min, gain_max, eps_blend)
    if abs(e_pos) > 1e-6
        val_ratio = -m * e_vel / e_pos;
    else
        val_ratio = 0;
    end
    val_ratio = max(min(val_ratio, gain_max), -gain_max);

    a_adaptive = max(val_ratio, gain_min);

    % 混合权重: 位置误差越小, 越倾向固定增益
    w = min(abs(e_pos) / eps_blend, 1.0);

    a = w * a_adaptive + (1 - w) * gain_min;
end
