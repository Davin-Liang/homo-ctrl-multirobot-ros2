%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4D 连续边界投影 + C++ omega_d 增益机制
%%
%%   编队策略: 纯径向投影 (与 demo_4d_cont 一致)
%%   增益计算: 移植 C++ calculate_klin (homo_controller_4d_cont.hpp:171-191)
%%
%%   C++ 增益公式:
%%     max_ratio = omega_d * m
%%     val  = clamp(-m * e_vel / e_pos, +-max_ratio)
%%     a    = max(val, max_ratio)
%%     结果: a = max_ratio 恒为常数 (clamp上界 == 下界)
%%
%%   对比:
%%     - demo_4d_cont:            纯径向 + MATLAB 自适应增益 (gain_min=4)
%%     - demo_4d_cont_tangent:    纯径向 + MATLAB 自适应增益 + 噪声
%%     - demo_4d_cont_omegad:     纯径向 + C++ omega_d 增益机制 + 噪声 (本文件)
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

%% ====== C++ omega_d 增益参数 ======
omega_d = 1.5;          % 期望阻尼带宽 (C++ default: 1.5)
%  max_ratio = omega_d * m = 3.0 (with m=2)
%  clamp(val, +-3) then max(val, 3) => a = 3 always
%  闭环特征值: s = -a/m = -1.5

%% ====== 测量噪声 ======
noise_pos_std = 0.2;
noise_vel_std = 0.7;
rng(42);

%% ====== 初始误差 (纯径向投影) ======
dpos = [x2(1)-x1(1); x2(2)-x1(2)];
r_dist = max(sqrt(dpos(1)^2 + dpos(2)^2), 1e-3);
d_proj = radius * dpos / r_dist;
e_pos = dpos - d_proj;
e = [e_pos; x2(3)-x1(3); x2(4)-x1(4)];

%% ====== 初始增益 (C++ 公式) ======
[a, va] = calc_gain_cpp(m, e(1), e(3), omega_d);
[b, vb] = calc_gain_cpp(m, e(2), e(4), omega_d);
k_lin = [a*(-2*a + a)/m, 0, -2*a, 0; 0, b*(-2*b + b)/m, 0, -2*b];

fprintf('=== 初始 === omega_d=%.2f m=%.1f max_ratio=%.2f ===\n', omega_d, m, omega_d*m);
fprintf('x1=[%.2f %.2f %.2f %.2f]  x2=[%.2f %.2f %.2f %.2f]\n', x1, x2);
fprintf('va=%.3f vb=%.3f -> a=%.2f b=%.2f  (constant gain)\n', va, vb, a, b);

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

    %% --- 带噪测量 ---
    x1_meas = x1 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];
    x2_meas = x2 + [noise_pos_std*randn(2,1); noise_vel_std*randn(2,1)];

    %% --- 纯径向投影误差 (基于带噪测量) ---
    dpos_meas = [x2_meas(1)-x1_meas(1); x2_meas(2)-x1_meas(2)];
    r_m = max(sqrt(dpos_meas(1)^2 + dpos_meas(2)^2), 1e-3);
    e_pos = dpos_meas - radius * dpos_meas / r_m;
    e = [e_pos; x2_meas(3)-x1_meas(3); x2_meas(4)-x1_meas(4)];

    %% --- C++ 增益公式 ---
    [a, ~] = calc_gain_cpp(m, e(1), e(3), omega_d);
    [b, ~] = calc_gain_cpp(m, e(2), e(4), omega_d);
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

    %% --- 日志 ---
    dpos_t = [x2(1)-x1(1); x2(2)-x1(2)];
    r_t = norm(dpos_t);
    e_t = [dpos_t - radius*dpos_t/max(r_t,1e-3); x2(3)-x1(3); x2(4)-x1(4)];
    t = t + h;
    tl = [tl t];  xl1 = [xl1 x1];  xl2 = [xl2 x2];
    ul1 = [ul1 u1];  ul2 = [ul2 u2];
    el = [el e_t];  ul = [ul u2];
    al = [al a];  bl = [bl b];

    if mod(step, print_step) == 0
        fprintf('t=%.2f | r=%.3f | err_meas=[%+.4f %+.4f] | a=%.2f b=%.2f | u2=[%+.3f %+.3f]\n', ...
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
title(sprintf('X-axis (\\omega_d=%.1f, gain=%.1f)', omega_d, omega_d*m));

figure(2); hold on;
plot(tl, xl1(2,:), 'r', 'LineWidth', 2); plot(tl, xl2(2,:), 'b', 'LineWidth', 2);
xlim([0 Tmax]); ylim([-1.5 1.5])
xlabel('$t(s)$','Interpreter','latex'); ylabel('$y$','Interpreter','latex');
legend('$y_1$','$y_2$','Interpreter','latex','Box','off'); grid on;
title(sprintf('Y-axis (\\omega_d=%.1f, gain=%.1f)', omega_d, omega_d*m));

figure(3); hold on;
plot(xl1(1,:), xl1(2,:), 'r', 'LineWidth', 2); plot(xl2(1,:), xl2(2,:), 'b', 'LineWidth', 2);
xlim([-1.5 2.5]); ylim([-1.5 1.5])
xlabel('$x$','Interpreter','latex'); ylabel('$y$','Interpreter','latex');
legend('$r_1$','$r_2$','Interpreter','latex','Box','off'); grid on;

figure(4); hold on;
plot(tl, ul2(1,:), 'r', 'LineWidth', 2); plot(tl, ul2(2,:), 'b', 'LineWidth', 2);
ylim([-30 20]); xlabel('$t$','Interpreter','latex'); ylabel('$u$','Interpreter','latex');
legend('$u_x$','$u_y$','Interpreter','latex','Box','off'); grid on;

figure(5); hold on;
dist_bnd = sqrt((xl2(1,:)-xl1(1,:)).^2 + (xl2(2,:)-xl1(2,:)).^2) - radius;
plot(tl, dist_bnd, 'b', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 1.5);
xlabel('$t(s)$','Interpreter','latex'); ylabel('dist to boundary','Interpreter','latex'); grid on;

figure(6); hold on;
plot(tl, E, 'r', 'LineWidth', 2); ylim([0 5]);
xlabel('$t(s)$','Interpreter','latex'); ylabel('$||e_{pos}||$','Interpreter','latex'); grid on;

figure(7); hold on;
plot(tl, al, 'r', 'LineWidth', 1.5); plot(tl, bl, 'b', 'LineWidth', 1.5);
yline(omega_d*m, 'k--'); yline(-omega_d*m, 'k:');
ylim([-omega_d*m*2, omega_d*m*2]);
xlabel('$t(s)$','Interpreter','latex'); ylabel('Gain','Interpreter','latex');
legend('$a$ (x)','$b$ (y)','Interpreter','latex','Box','off');
title(sprintf('Gain (\\omega_d=%.1f, max\\_ratio=%.1f)', omega_d, omega_d*m));
grid on;

%% ====== Y 轴总结 ======
y_err = xl2(2,:) - xl1(2,:);
fprintf('\n===== Y-axis (omega_d=%.1f, noise: pos=%.3f vel=%.3f) =====\n', ...
    omega_d, noise_pos_std, noise_vel_std);
fprintf('  Gain const = %.1f  (omega_d * m)\n', omega_d*m);
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
%%  C++ calculate_klin 移植
%%
%%  max_ratio = omega_d * m
%%  val  = clamp(-m * e_vel / e_pos, +-max_ratio)
%%  a    = max(val, max_ratio)
%%
%%  结果: a = max_ratio 恒为常数
%%  闭环极点: s^2 + 2a*s/m + a^2/m^2 = 0 => s = -a/m = -omega_d (重根)
%% ========================================================================
function [a, val] = calc_gain_cpp(m, e_pos, e_vel, omega_d)
    max_ratio = omega_d * m;
    if abs(e_pos) > 1e-6
        val = -m * e_vel / e_pos;
    else
        val = 0;
    end
    val = max(min(val, max_ratio), -max_ratio);
    a = max(val, max_ratio);   % = max_ratio 恒成立
end
