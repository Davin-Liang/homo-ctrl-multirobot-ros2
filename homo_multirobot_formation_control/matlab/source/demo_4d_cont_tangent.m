%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4D 连续边界投影编队控制 (pure radial projection + measurement noise)
%%
%%   与 demo_4d_cont.m 算法一致 (纯径向投影), 加入测量噪声模拟 ROS 链路
%%
%%   e_pos = dpos_meas - radius * dpos_meas / |dpos_meas|
%%
%% 文件说明:
%%   - demo_4d_cont:               纯径向投影 (无噪声)
%%   - demo_4d_cont_tangent:       纯径向投影 + 噪声 (本文件)
%%   - demo_4d_cont_tangent_noise: 固定角度切向修正 + 噪声
%%
%% 运行条件: m=2, radius=1, h=0.01, Tmax=30
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

%% ====== 控制参数 ======
gain_min  = 4;
gain_max  = 40;
eps_blend = 0.15;

%% ====== 测量噪声 (模拟 ROS EKF+TF 链路) ======
noise_pos_std = 0.1;    % 位置噪声标准差 (m) 0.1
noise_vel_std = 0.7;    % 速度噪声标准差 (m/s) 0.7
rng(42);

%% ====== 初始误差 (纯径向投影) ======
dpos = [x2(1)-x1(1); x2(2)-x1(2)];
r_dist = max(sqrt(dpos(1)^2 + dpos(2)^2), 1e-3);
d_proj = radius * dpos / r_dist;
e_pos = dpos - d_proj;
e = [e_pos; x2(3)-x1(3); x2(4)-x1(4)];

%% ====== 初始增益 ======
[~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
[~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);
k_lin = [a*( -2*a + a)/m, 0, -2*a, 0; 0, b*(-2*b + b)/m, 0, -2*b];

fprintf('=== 初始状态 ===\n');
fprintf('x1=[%.4f %.4f %.4f %.4f]  x2=[%.4f %.4f %.4f %.4f]\n', x1, x2);
fprintf('r_dist=%.4f  a=%.2f b=%.2f\n', norm(dpos), a, b);

%% ====== LPC -> HPC ======
[~, G0, P, nu_min, ~] = lpc2hpc(A, B, k_lin);
nu = nu_min;
Gd = eye(4) + nu * G0;
fprintf('nu=%.4f\n\n', nu);

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

    %% --- 带噪测量 (模拟 ROS EKF+TF) ---
    x1_meas = x1 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];
    x2_meas = x2 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];

    %% --- 纯径向投影误差 (基于带噪测量) ---
    dpos_meas = [x2_meas(1)-x1_meas(1); x2_meas(2)-x1_meas(2)];
    r_m = max(sqrt(dpos_meas(1)^2 + dpos_meas(2)^2), 1e-3);
    e_pos = dpos_meas - radius * dpos_meas / r_m;
    e = [e_pos; x2_meas(3)-x1_meas(3); x2_meas(4)-x1_meas(4)];

    %% --- 自适应增益 ---
    [~, a] = smooth_gain(m, e(1), e(3), gain_min, gain_max, eps_blend);
    [~, b] = smooth_gain(m, e(2), e(4), gain_min, gain_max, eps_blend);
    k_lin = [a*(-2*a + a)/m, 0, -2*a, 0; 0, b*(-2*b + b)/m, 0, -2*b];

    %% --- 重算 HPC ---
    [~, G0, P, nu_min, ~] = lpc2hpc(A, B, k_lin);
    nu = nu_min;
    Gd = eye(4) + nu * G0;

    %% --- HPC 控制律 ---
    nx = hnorm(e, Gd, P);
    c = max(min(1, nx), 0.1);
    u2 = c^(1 + nu) * k_lin * expm(Gd * (1 - log(c))) * e;

    %% --- 跟随者动力学 (clean) ---
    x2 = x2 + h * (A * x2 + B * u2);

    %% --- 日志 (clean 真值) ---
    dpos_t = [x2(1)-x1(1); x2(2)-x1(2)];
    r_t = norm(dpos_t);
    e_t = [dpos_t - radius*dpos_t/max(r_t,1e-3); x2(3)-x1(3); x2(4)-x1(4)];
    t = t + h;
    tl = [tl t];  xl1 = [xl1 x1];  xl2 = [xl2 x2];
    ul1 = [ul1 u1];  ul2 = [ul2 u2];
    el = [el e_t];  ul = [ul u2];
    al = [al a];  bl = [bl b];

    if mod(step, print_step) == 0
        fprintf('t=%.2f | r_true=%.3f | err_meas=[%+.4f %+.4f] | a=%.2f b=%.2f | u2=[%+.3f %+.3f]\n', ...
            t, r_t, e(1), e(2), a, b, u2(1), u2(2));
    end
end

fprintf('\n=== 最终 === r=%.4f dev=%.4f  x2y=%.4f\n', ...
    norm([x2(1)-x1(1); x2(2)-x1(2)]), ...
    norm([x2(1)-x1(1); x2(2)-x1(2)]) - radius, x2(2));

%% ========================================================================
E = sqrt(el(1,:).^2 + el(2,:).^2);

figure(1); hold on;
plot(tl, xl1(1,:), 'r', 'LineWidth', 2); plot(tl, xl2(1,:), 'b', 'LineWidth', 2);
xlim([0 Tmax]); xlabel('$t(s)$','Interpreter','latex'); ylabel('$x$','Interpreter','latex');
legend('$x_1$','$x_2$','Interpreter','latex','Box','off'); grid on;

figure(2); hold on;
plot(tl, xl1(2,:), 'r', 'LineWidth', 2); plot(tl, xl2(2,:), 'b', 'LineWidth', 2);
xlim([0 Tmax]); ylim([-1.5 1.5])
xlabel('$t(s)$','Interpreter','latex'); ylabel('$y$','Interpreter','latex');
legend('$y_1$','$y_2$','Interpreter','latex','Box','off'); grid on;

figure(3); hold on;
plot(xl1(1,:), xl1(2,:), 'r', 'LineWidth', 2); plot(xl2(1,:), xl2(2,:), 'b', 'LineWidth', 2);
xlim([-1.5 2.5]); ylim([-1.5 1.5])
xlabel('$x$','Interpreter','latex'); ylabel('$y$','Interpreter','latex');
legend('$r_1$','$r_2$','Interpreter','latex','Box','off'); grid on;

figure(4); hold on;
plot(tl, ul2(1,:), 'r', 'LineWidth', 2); plot(tl, ul2(2,:), 'b', 'LineWidth', 2);
ylim([-15 10]); xlabel('$t$','Interpreter','latex'); ylabel('$u$','Interpreter','latex');
legend('$u_x$','$u_y$','Interpreter','latex','Box','off'); grid on;

figure(5); hold on;
dist_bnd = sqrt((xl2(1,:)-xl1(1,:)).^2 + (xl2(2,:)-xl1(2,:)).^2) - radius;
plot(tl, dist_bnd, 'b', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 1.5);
xlabel('$t(s)$','Interpreter','latex'); ylabel('dist to boundary (m)','Interpreter','latex'); grid on;

figure(6); hold on;
plot(tl, E, 'r', 'LineWidth', 2); ylim([0 3.5]);
xlabel('$t(s)$','Interpreter','latex'); ylabel('$||e_{pos}||$','Interpreter','latex'); grid on;

figure(7); hold on;
plot(tl, al, 'r', 'LineWidth', 1.5); plot(tl, bl, 'b', 'LineWidth', 1.5);
yline(gain_min, 'k--'); yline(gain_max, 'k:');
xlabel('$t(s)$','Interpreter','latex'); ylabel('Gain','Interpreter','latex');
legend('$a$ (x)','$b$ (y)','Interpreter','latex','Box','off'); grid on;

%% ====== Y 轴总结 ======
y_err = xl2(2,:) - xl1(2,:);
fprintf('\n===== Y-axis Tracking (radial, noise: pos=%.3f vel=%.3f) =====\n', noise_pos_std, noise_vel_std);
fprintf('  RMS |Y| err: %.4f  Max |Y| err: %.4f  Mean Y err: %.4f\n', ...
    sqrt(mean(y_err.^2)), max(abs(y_err)), mean(y_err));
check_t = [5 10 15 20 25 30];
for ti = check_t
    [~, idx] = min(abs(tl - ti));
    fprintf('    t=%-2.0f  y_lead=%+.4f  y_follow=%+.4f  err=%+.4f\n', ...
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
