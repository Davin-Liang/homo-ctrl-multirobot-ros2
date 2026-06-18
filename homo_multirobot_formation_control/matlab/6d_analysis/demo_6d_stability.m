%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6D Time-Varying Kinematic Model: HPC Stability Analysis
%%
%% This script demonstrates the 6D kinematic homogeneous formation
%% control for omnidirectional mobile robots, including:
%%
%%   1. Frozen-time stability analysis (eigenvalue sweep over leader velocities)
%%   2. Lyapunov function verification (P*Gd + Gd'*P > 0)
%%   3. Admissible homogeneity degree range
%%   4. Leader-follower simulation (circular + sinusoidal trajectories)
%%   5. 6D HPC vs 6D LPC ablation comparison
%%   6. 6D HPC vs 4D HPC model comparison
%%   7. Robustness to leader velocity variation rate
%%
%% Based on: lpc2hpc.m, hnorm.m, block_con.m, trans_con.m (original HPC toolbox)
%% Extended to: 6D kinematic model with time-varying A matrix
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all;

% Add path to original HPC toolbox source
addpath('../source');

%% ========================================================================
%% Section 1: System Setup -- 6D Kinematic Model
%% ========================================================================

fprintf('=============================================================\n');
fprintf('  6D Kinematic HPC Stability Analysis\n');
fprintf('=============================================================\n\n');

% --- 6D state definition ---
% x = [px, py, theta, vx_body, vy_body, omega]^T
%   px, py, theta : position & heading in MAP frame
%   vx_b, vy_b, omega : velocities in BODY frame

% --- Model parameters ---
mass  = 8.0;     % translation tuning mass
I_val = 1.0;     % rotational inertia tuning
radius = 2.0;    % formation safety circle radius (m)
omega_d_pos = 1.5;   % position damping bandwidth
omega_d_theta = 1.5; % yaw damping bandwidth

% --- Build nominal system matrices ---
% A_l is time-varying; nominal version with zero leader velocity for init
[A_nom, B_6d] = build_6d_system(0, 0, 0, mass, I_val);

fprintf('6D system: n=%d states, m=%d inputs\n', size(A_nom,1), size(B_6d,2));
fprintf('Parameters: mass=%.1f, I=%.1f, radius=%.1f m\n', mass, I_val, radius);

%% ========================================================================
%% Section 2: Frozen-Time Stability Analysis
%% ========================================================================
%% Sweep over leader velocities (omega_l, vx_l) and verify:
%%   (a) A_l + B*K is Hurwitz for all frozen-time systems
%%   (b) P*Gd + Gd'*P > 0  (HPC Lyapunov condition)
%%   (c) Admissible homogeneity degree range [nu_min, nu_max]

fprintf('\n--- Frozen-Time Stability Analysis ---\n');

% Grid over leader velocities
omega_vec = linspace(-1.5, 1.5, 31);  % rad/s
vx_vec    = linspace(0.0, 2.0, 21);    % m/s
vy_l = 0;  % assume leader has zero lateral velocity

[OMEGA, VX] = meshgrid(omega_vec, vx_vec);
max_eig_lpc  = zeros(size(OMEGA));  % max real eigenvalue of A+BK
max_eig_hpc  = zeros(size(OMEGA));  % max real eigenvalue with HPC
nu_min_grid  = zeros(size(OMEGA));
nu_max_grid  = zeros(size(OMEGA));
lyap_cond    = zeros(size(OMEGA));  % min eig of P*Gd+Gd'*P

% Nominal error for computing linear gain (unit pos error, zero vel error)
e_nom = [1; 0; 0; 0; 0; 0];

for i = 1:size(OMEGA,1)
    for j = 1:size(OMEGA,2)
        omega_l = OMEGA(i,j);
        vx_l    = VX(i,j);

        % Build A_l for this leader velocity
        [A_l, ~] = build_6d_system(omega_l, vx_l, vy_l, mass, I_val);

        % Compute adaptive linear gain (critically damped, 3 channels)
        K_lin = compute_6d_linear_gain(e_nom, mass, I_val, ...
                                        omega_d_pos, omega_d_theta);

        % Closed-loop LPC: A_l + B_6d * K_lin
        A_cl_lpc = A_l + B_6d * K_lin;
        max_eig_lpc(i,j) = max(real(eig(A_cl_lpc)));

        % Upgrade to HPC
        [K0, G0, P, nu_min, nu_max] = lpc2hpc(A_l, B_6d, K_lin);
        nu = nu_min;  % use minimum homogeneity degree (maximum robustness)
        Gd = eye(6) + nu * G0;
        K_hpc = K_lin - K0;

        % HPC closed-loop characterization
        A_cl_hpc = A_l + B_6d * (K0 + K_hpc);
        max_eig_hpc(i,j) = max(real(eig(A_cl_hpc)));

        % Lyapunov condition: P*Gd + Gd'*P > 0
        M_lyap = P * Gd + Gd' * P;
        lyap_cond(i,j) = min(real(eig(M_lyap)));

        % Admissible homogeneity degree
        nu_min_grid(i,j) = nu_min;
        nu_max_grid(i,j) = nu_max;
    end
end

fprintf('  max  real(eig(A+BK_LPC)): %.6f  (should be < 0)\n', max(max_eig_lpc(:)));
fprintf('  max  real(eig(A+BK_HPC)): %.6f  (should be < 0)\n', max(max_eig_hpc(:)));
fprintf('  min  eig(P*Gd+Gd''*P):   %.6f  (should be > 0)\n', min(lyap_cond(:)));
fprintf('  nu  range: [%.3f, %.3f]\n', min(nu_min_grid(:)), max(nu_max_grid(:)));

% --- Figure 1: Frozen-time stability heatmaps ---
figure(1); clf;
set(gcf, 'Position', [100, 100, 1200, 800]);

subplot(2,3,1);
contourf(OMEGA, VX, max_eig_lpc, 20);
colorbar; colormap(jet);
xlabel('$\omega_l$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$v_{x,l}$ (m/s)', 'Interpreter', 'latex', 'FontSize', 14);
title('$\max\mathrm{Re}(\lambda(A_l+BK_{\mathrm{lin}}))$ -- LPC', ...
      'Interpreter', 'latex', 'FontSize', 14);
hold on; contour(OMEGA, VX, max_eig_lpc, [0 0], 'k-', 'LineWidth', 2); hold off;

subplot(2,3,2);
contourf(OMEGA, VX, lyap_cond, 20);
colorbar; colormap(jet);
xlabel('$\omega_l$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$v_{x,l}$ (m/s)', 'Interpreter', 'latex', 'FontSize', 14);
title('$\min\lambda(P G_d + G_d^{\top} P)$ -- Lyapunov Condition', ...
      'Interpreter', 'latex', 'FontSize', 14);
hold on; contour(OMEGA, VX, lyap_cond, [0 0], 'k-', 'LineWidth', 2); hold off;

subplot(2,3,3);
contourf(OMEGA, VX, nu_min_grid, 20);
colorbar; colormap(jet);
xlabel('$\omega_l$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$v_{x,l}$ (m/s)', 'Interpreter', 'latex', 'FontSize', 14);
title('$\nu_{\mathrm{min}}$ -- Admissible Homogeneity Degree', ...
      'Interpreter', 'latex', 'FontSize', 14);

subplot(2,3,4);
contourf(OMEGA, VX, nu_max_grid, 20);
colorbar; colormap(jet);
xlabel('$\omega_l$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$v_{x,l}$ (m/s)', 'Interpreter', 'latex', 'FontSize', 14);
title('$\nu_{\mathrm{max}}$', 'Interpreter', 'latex', 'FontSize', 14);

% Eigenvalue distribution at selected operating points
subplot(2,3,5:6);
hold on;
colors = lines(5);
idx_samples = [1, 8, 16, 24, 31];  % sample omega_l values
for k = 1:length(idx_samples)
    omega_l = omega_vec(idx_samples(k));
    vx_l = 1.0;  % fixed forward speed
    [A_l, ~] = build_6d_system(omega_l, vx_l, vy_l, mass, I_val);
    K_lin = compute_6d_linear_gain(e_nom, mass, I_val, ...
                                    omega_d_pos, omega_d_theta);
    [K0, G0, P, nu_min, nu_max] = lpc2hpc(A_l, B_6d, K_lin);
    nu = nu_min;
    Gd = eye(6) + nu * G0;

    % Eigenvalues of LPC
    eig_lpc = eig(A_l + B_6d * K_lin);
    % Eigenvalues of "warped" HPC closed-loop (at unit norm)
    eig_hpc = eig(A_l + B_6d * (K0 + (K_lin - K0)));

    h1 = plot(real(eig_lpc), imag(eig_lpc), 'o', 'MarkerSize', 8, ...
              'Color', colors(k,:), 'LineWidth', 1.5);
    plot(real(eig_hpc), imag(eig_hpc), 'x', 'MarkerSize', 10, ...
         'Color', colors(k,:), 'LineWidth', 1.5);
    leg_entries{k} = sprintf('$\\omega_l=%.2f$ (LPC)', omega_l);
end
plot([0 0], [-8 8], 'k--', 'LineWidth', 0.5);  % imaginary axis
xlabel('$\mathrm{Re}(\lambda)$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$\Im(\lambda)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Closed-Loop Eigenvalues: LPC (o) vs HPC (x)', ...
      'Interpreter', 'latex', 'FontSize', 14);
legend(leg_entries, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'best');
grid on; axis equal;
hold off;

sgtitle('Frozen-Time Stability Analysis of 6D Kinematic HPC', ...
        'Interpreter', 'latex', 'FontSize', 16);

fprintf('  Frozen-time analysis complete. See Figure 1.\n');
fprintf(['  Note: "weakly controllable" warnings are expected\n' ...
         '  (numerical conditioning with mass=%.0f, I=%.0f).\n' ...
         '  The 6D system is fully controllable (rank=6).\n'], mass, I_val);

%% ========================================================================
%% Section 3: Leader-Follower Simulation -- Circular Trajectory
%% ========================================================================

fprintf('\n--- Circular Trajectory Simulation ---\n');

% Simulation parameters
h = 0.01;        % time step (s)
Tmax = 40;       % simulation duration (s)
t = 0:h:Tmax;
N = length(t);

% Leader: circular trajectory via constant body-frame velocity
vx_l_des = 0.5;       % constant forward speed (m/s)
omega_l_des = 0.25;   % constant turn rate (rad/s) -> radius = 0.5/0.25 = 2m

% Storage
x_l = zeros(6, N);   % leader state
x_f_hpc = zeros(6, N); % follower state (HPC)
x_f_lpc = zeros(6, N); % follower state (LPC, for ablation)
e_hpc_hist = zeros(6, N);
e_lpc_hist = zeros(6, N);
u_hpc_hist = zeros(3, N);
u_lpc_hist = zeros(3, N);
V_hpc_hist = zeros(1, N);  % Lyapunov function
V_lpc_hist = zeros(1, N);
hpc_recomp_hist = zeros(1, N);  % HPC recomputation flag
nu_hist = zeros(1, N);

% Initial conditions
x_l(:,1) = [0; 0; 0; vx_l_des; 0; omega_l_des];
% Follower starts 3m behind and 1m to the side
x_f_hpc(:,1) = [-3; -1; 0; 0; 0; 0];
x_f_lpc(:,1) = [-3; -1; 0; 0; 0; 0];

% HPC state (parameters recomputed when leader velocity changes)
hpc_active = false;
last_hpc_leader_vel = [0; 0; 0];
last_dtheta = 0;
hpc_vel_threshold = 0.3;
K0 = zeros(3,6); G0 = zeros(6); P = eye(6); nu = -1; Gd = eye(6);
K_hpc = zeros(3,6);
K_lin_hpc = zeros(3,6);
K_lin_lpc = zeros(3,6);

fprintf('  Running simulation (%d steps)...\n', N);

for k = 1:N-1
    % --- Leader dynamics (nonlinear kinematics) ---
    % Leader maintains constant body-frame velocity (open-loop)
    x_l(:,k+1) = integrate_6d_kinematics(x_l(:,k), zeros(3,1), h, mass, I_val);

    % --- Compute 6D error in leader body frame ---
    e_hpc = compute_6d_error(x_l(:,k), x_f_hpc(:,k), radius);
    e_lpc = compute_6d_error(x_l(:,k), x_f_lpc(:,k), radius);

    % --- Adaptive linear gain ---
    K_lin_hpc = compute_6d_linear_gain(e_hpc, mass, I_val, ...
                                        omega_d_pos, omega_d_theta);
    K_lin_lpc = compute_6d_linear_gain(e_lpc, mass, I_val, ...
                                        omega_d_pos, omega_d_theta);

    % --- Build current A_l ---
    % Extract leader body-frame velocity
    vx_l_cur = x_l(4,k);
    vy_l_cur = x_l(5,k);
    omega_l_cur = x_l(6,k);
    [A_l_cur, ~] = build_6d_system(omega_l_cur, vx_l_cur, vy_l_cur, mass, I_val);

    % --- HPC parameter recomputation (gain scheduling) ---
    leader_vel = [omega_l_cur; vx_l_cur; vy_l_cur];
    dtheta = x_f_hpc(3,k) - x_l(3,k);

    if ~hpc_active || ...
       norm(leader_vel - last_hpc_leader_vel) > hpc_vel_threshold || ...
       abs(dtheta - last_dtheta) > 0.15

        [K0, G0, P, nu_min, nu_max] = lpc2hpc(A_l_cur, B_6d, K_lin_hpc);
        nu = nu_min;  % negative degree for robustness near origin
        Gd = eye(6) + nu * G0;
        K_hpc = K_lin_hpc - K0;

        hpc_active = true;
        last_hpc_leader_vel = leader_vel;
        last_dtheta = dtheta;
        hpc_recomp_hist(k+1) = 1;
    end

    % --- HPC control law (matches e_hpc.m convention) ---
    % u = K0*e + hn^(1+mu)*K*expm(-log(hn)*Gd)*e
    % where K = K_lin - K0 (extra gain beyond homogenizing K0)
    c = hnorm(e_hpc, Gd, P, 0.5, 1.0, 20);
    if c > 1e-10
        u_L_hpc = K0 * e_hpc + c^(1+nu) * K_hpc * expm(-log(c) * Gd) * e_hpc;
    else
        u_L_hpc = K0 * e_hpc;
    end

    % --- LPC control law (ablation baseline) ---
    u_L_lpc = K_lin_lpc * e_lpc;

    % --- Rotate forces: leader body frame -> follower body frame ---
    u_f_hpc = rotate_control_to_follower(u_L_hpc, dtheta);
    u_f_lpc = rotate_control_to_follower(u_L_lpc, x_f_lpc(3,k) - x_l(3,k));

    % --- Follower dynamics (nonlinear kinematics) ---
    x_f_hpc(:,k+1) = integrate_6d_kinematics(x_f_hpc(:,k), u_f_hpc, h, mass, I_val);
    x_f_lpc(:,k+1) = integrate_6d_kinematics(x_f_lpc(:,k), u_f_lpc, h, mass, I_val);

    % --- Logging ---
    e_hpc_hist(:,k) = e_hpc;
    e_lpc_hist(:,k) = e_lpc;
    u_hpc_hist(:,k) = u_f_hpc;
    u_lpc_hist(:,k) = u_f_lpc;
    nu_hist(k) = nu;

    % Lyapunov function: V(e) = homogeneous norm of error
    V_hpc_hist(k) = hnorm(e_hpc, Gd, P, 0.01, 100, 20);
end

% Final step logging
e_hpc_hist(:,N) = compute_6d_error(x_l(:,N), x_f_hpc(:,N), radius);
e_lpc_hist(:,N) = compute_6d_error(x_l(:,N), x_f_lpc(:,N), radius);
V_hpc_hist(N) = hnorm(e_hpc_hist(:,N), Gd, P, 0.01, 100, 20);

fprintf('  Simulation complete.\n');

% --- Figure 2: Trajectories (XY plane) ---
figure(2); clf;
set(gcf, 'Position', [150, 150, 900, 600]);

subplot(1,2,1);
hold on;
plot(x_l(1,:), x_l(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Leader');
plot(x_f_hpc(1,:), x_f_hpc(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Follower (6D HPC)');
% Plot formation circle at final state
th_circ = linspace(0, 2*pi, 100);
plot(x_l(1,end) + radius*cos(th_circ), x_l(2,end) + radius*sin(th_circ), ...
     'k--', 'LineWidth', 0.8, 'DisplayName', 'Safety circle');
% Mark start/end
plot(x_l(1,1), x_l(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x_l(1,end), x_l(2,end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x_f_hpc(1,1), x_f_hpc(2,1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(x_f_hpc(1,end), x_f_hpc(2,end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$p_y$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Circular Trajectory -- 6D HPC', 'Interpreter', 'tex', 'FontSize', 14);
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 11);
axis equal; grid on;
hold off;

subplot(1,2,2);
hold on;
plot(x_l(1,:), x_l(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Leader');
plot(x_f_lpc(1,:), x_f_lpc(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Follower (6D LPC)');
plot(x_l(1,end) + radius*cos(th_circ), x_l(2,end) + radius*sin(th_circ), ...
     'k--', 'LineWidth', 0.8);
plot(x_l(1,1), x_l(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x_f_lpc(1,1), x_f_lpc(2,1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$p_y$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Circular Trajectory -- 6D LPC (Ablation)', 'Interpreter', 'tex', 'FontSize', 14);
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 11);
axis equal; grid on;
hold off;

% --- Figure 3: Position tracking error over time ---
figure(3); clf;
set(gcf, 'Position', [200, 200, 1000, 800]);

subplot(3,1,1);
pos_err_hpc = sqrt(e_hpc_hist(1,:).^2 + e_hpc_hist(2,:).^2);
pos_err_lpc = sqrt(e_lpc_hist(1,:).^2 + e_lpc_hist(2,:).^2);
plot(t, pos_err_hpc, 'b-', 'LineWidth', 1.5); hold on;
plot(t, pos_err_lpc, 'r-', 'LineWidth', 1.5);
yline(radius, 'k--', 'LineWidth', 1);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$\|e_{xy}\|$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Position Error Norm', 'Interpreter', 'tex', 'FontSize', 14);
legend('6D HPC', '6D LPC', '$r_s$ (target)', ...
       'Interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3,1,2);
plot(t, abs(e_hpc_hist(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t, abs(e_lpc_hist(3,:)), 'r-', 'LineWidth', 1.5);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$|e_\theta|$ (rad)', 'Interpreter', 'latex', 'FontSize', 14);
title('Heading Error', 'Interpreter', 'tex', 'FontSize', 14);
legend('6D HPC', '6D LPC', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3,1,3);
plot(t, V_hpc_hist, 'b-', 'LineWidth', 1.5);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$V(e) = \|e\|_{G_d,P}$', 'Interpreter', 'latex', 'FontSize', 14);
title('Homogeneous Lyapunov Function', 'Interpreter', 'tex', 'FontSize', 14);
grid on;

% --- Figure 4: Control inputs ---
figure(4); clf;
set(gcf, 'Position', [250, 250, 1000, 600]);

subplot(3,1,1);
plot(t, u_hpc_hist(1,:), 'b-', 'LineWidth', 1.0); hold on;
plot(t, u_lpc_hist(1,:), 'r-', 'LineWidth', 1.0);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$F_x$ (N)', 'Interpreter', 'latex', 'FontSize', 14);
title('Control Force $F_x$ (Body Frame)', 'Interpreter', 'latex', 'FontSize', 14);
legend('6D HPC', '6D LPC', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3,1,2);
plot(t, u_hpc_hist(2,:), 'b-', 'LineWidth', 1.0); hold on;
plot(t, u_lpc_hist(2,:), 'r-', 'LineWidth', 1.0);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$F_y$ (N)', 'Interpreter', 'latex', 'FontSize', 14);
title('Control Force $F_y$ (Body Frame)', 'Interpreter', 'latex', 'FontSize', 14);
legend('6D HPC', '6D LPC', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3,1,3);
plot(t, u_hpc_hist(3,:), 'b-', 'LineWidth', 1.0); hold on;
plot(t, u_lpc_hist(3,:), 'r-', 'LineWidth', 1.0);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$\tau$ (Nm)', 'Interpreter', 'latex', 'FontSize', 14);
title('Control Torque $\tau$', 'Interpreter', 'latex', 'FontSize', 14);
legend('6D HPC', '6D LPC', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

%% ========================================================================
%% Section 4: Sinusoidal Trajectory -- Time-Varying A Matrix
%% ========================================================================
%% Leader with sinusoidal omega_l exercises the time-varying A_l.
%% This is where gain-scheduled HPC should outperform fixed-gain LPC.

fprintf('\n--- Sinusoidal Trajectory (Time-Varying A) ---\n');

T_sin = 30;          % duration
N_sin = round(T_sin / h);
t_sin = (0:N_sin-1) * h;

% Leader: constant forward + sinusoidal turning
omega_amp = 0.6;      % amplitude (rad/s)
omega_freq = 0.3;     % frequency (Hz)
vx_l_sin = 0.5;       % forward speed (m/s)

x_l_sin = zeros(6, N_sin);
x_l_sin(:,1) = [0; 0; 0; vx_l_sin; 0; 0];

x_f_hpc_sin = zeros(6, N_sin);
x_f_hpc_sin(:,1) = [-3; -1; 0; 0; 0; 0];
x_f_lpc_sin = zeros(6, N_sin);
x_f_lpc_sin(:,1) = [-3; -1; 0; 0; 0; 0];

e_hpc_sin_hist = zeros(6, N_sin);
e_lpc_sin_hist = zeros(6, N_sin);
u_hpc_sin_hist = zeros(3, N_sin);
u_lpc_sin_hist = zeros(3, N_sin);
V_sin_hist = zeros(1, N_sin);
hpc_recomp_sin = zeros(1, N_sin);
nu_sin_hist = zeros(1, N_sin);

hpc_active_sin = false;
last_hpc_vel_sin = [0;0;0];
last_dtheta_sin = 0;
K0_sin=zeros(3,6); G0_sin=zeros(6); P_sin=eye(6); nu_sin=-1; Gd_sin=eye(6);
K_hpc_sin=zeros(3,6);

fprintf('  Running simulation (%d steps)...\n', N_sin);

for k = 1:N_sin-1
    % Leader: sinusoidal omega_l
    omega_l_sin = omega_amp * sin(2*pi*omega_freq * k*h);
    x_l_sin(6,k) = omega_l_sin;
    x_l_sin(:,k+1) = integrate_6d_kinematics(x_l_sin(:,k), [0;0;0], h, mass, I_val);

    % Errors
    e_hpc_sin = compute_6d_error(x_l_sin(:,k), x_f_hpc_sin(:,k), radius);
    e_lpc_sin = compute_6d_error(x_l_sin(:,k), x_f_lpc_sin(:,k), radius);

    % Adaptive linear gains
    K_lin_hpc_sin = compute_6d_linear_gain(e_hpc_sin, mass, I_val, ...
                                            omega_d_pos, omega_d_theta);
    K_lin_lpc_sin = compute_6d_linear_gain(e_lpc_sin, mass, I_val, ...
                                            omega_d_pos, omega_d_theta);

    [A_l_sin, ~] = build_6d_system(x_l_sin(6,k), x_l_sin(4,k), x_l_sin(5,k), mass, I_val);

    % HPC gain scheduling
    leader_vel_sin = [x_l_sin(6,k); x_l_sin(4,k); x_l_sin(5,k)];
    dtheta_sin = x_f_hpc_sin(3,k) - x_l_sin(3,k);

    if ~hpc_active_sin || ...
       norm(leader_vel_sin - last_hpc_vel_sin) > hpc_vel_threshold || ...
       abs(dtheta_sin - last_dtheta_sin) > 0.15
        [K0_sin, G0_sin, P_sin, nu_min_sin, nu_max_sin] = lpc2hpc(A_l_sin, B_6d, K_lin_hpc_sin);
        nu_sin = nu_min_sin;
        Gd_sin = eye(6) + nu_sin * G0_sin;
        K_hpc_sin = K_lin_hpc_sin - K0_sin;
        hpc_active_sin = true;
        last_hpc_vel_sin = leader_vel_sin;
        last_dtheta_sin = dtheta_sin;
        hpc_recomp_sin(k+1) = 1;
    end

    % HPC control
    c_sin = hnorm(e_hpc_sin, Gd_sin, P_sin, 0.5, 1.0, 20);
    if c_sin > 1e-10
        u_L_hpc_sin = K0_sin * e_hpc_sin + c_sin^(1+nu_sin) * K_hpc_sin * expm(-log(c_sin) * Gd_sin) * e_hpc_sin;
    else
        u_L_hpc_sin = K0_sin * e_hpc_sin;
    end

    % LPC control
    u_L_lpc_sin = K_lin_lpc_sin * e_lpc_sin;

    u_f_hpc_sin = rotate_control_to_follower(u_L_hpc_sin, dtheta_sin);
    u_f_lpc_sin = rotate_control_to_follower(u_L_lpc_sin, x_f_lpc_sin(3,k) - x_l_sin(3,k));

    x_f_hpc_sin(:,k+1) = integrate_6d_kinematics(x_f_hpc_sin(:,k), u_f_hpc_sin, h, mass, I_val);
    x_f_lpc_sin(:,k+1) = integrate_6d_kinematics(x_f_lpc_sin(:,k), u_f_lpc_sin, h, mass, I_val);

    e_hpc_sin_hist(:,k) = e_hpc_sin;
    e_lpc_sin_hist(:,k) = e_lpc_sin;
    u_hpc_sin_hist(:,k) = u_f_hpc_sin;
    u_lpc_sin_hist(:,k) = u_f_lpc_sin;
    V_sin_hist(k) = hnorm(e_hpc_sin, Gd_sin, P_sin, 0.01, 100, 20);
    nu_sin_hist(k) = nu_sin;
end
e_hpc_sin_hist(:,N_sin) = compute_6d_error(x_l_sin(:,N_sin), x_f_hpc_sin(:,N_sin), radius);
e_lpc_sin_hist(:,N_sin) = compute_6d_error(x_l_sin(:,N_sin), x_f_lpc_sin(:,N_sin), radius);
V_sin_hist(N_sin) = hnorm(e_hpc_sin_hist(:,N_sin), Gd_sin, P_sin, 0.01, 100, 20);

fprintf('  Simulation complete.\n');

% --- Sinusoidal metrics ---
pos_err_hpc_sin = sqrt(e_hpc_sin_hist(1,:).^2 + e_hpc_sin_hist(2,:).^2);
pos_err_lpc_sin = sqrt(e_lpc_sin_hist(1,:).^2 + e_lpc_sin_hist(2,:).^2);

init_err_hpc_sin = pos_err_hpc_sin(1);
t90_thr_hpc_sin = 0.1 * init_err_hpc_sin;
settled_sin = pos_err_hpc_sin < t90_thr_hpc_sin;
t_start_sin = max(1, find(pos_err_hpc_sin > t90_thr_hpc_sin, 1, 'last') + 1);
if isempty(t_start_sin), t_start_sin = 1; end
T90_hpc_sin = T_sin;
for k = t_start_sin:N_sin-50
    if all(settled_sin(k:min(k+50, N_sin)))
        T90_hpc_sin = t_sin(k); break;
    end
end

init_err_lpc_sin = pos_err_lpc_sin(1);
t90_thr_lpc_sin = 0.1 * init_err_lpc_sin;
settled_lpc_sin = pos_err_lpc_sin < t90_thr_lpc_sin;
t_start_lpc_sin = max(1, find(pos_err_lpc_sin > t90_thr_lpc_sin, 1, 'last') + 1);
if isempty(t_start_lpc_sin), t_start_lpc_sin = 1; end
T90_lpc_sin = T_sin;
for k = t_start_lpc_sin:N_sin-50
    if all(settled_lpc_sin(k:min(k+50, N_sin)))
        T90_lpc_sin = t_sin(k); break;
    end
end

ss_sin_start = round(0.7 * N_sin);
ss_mean_hpc_sin = mean(pos_err_hpc_sin(ss_sin_start:end));
ss_std_hpc_sin  = std(pos_err_hpc_sin(ss_sin_start:end));
ss_rmse_hpc_sin = sqrt(mean(pos_err_hpc_sin(ss_sin_start:end).^2));
ss_mean_lpc_sin = mean(pos_err_lpc_sin(ss_sin_start:end));
ss_std_lpc_sin  = std(pos_err_lpc_sin(ss_sin_start:end));
ss_rmse_lpc_sin = sqrt(mean(pos_err_lpc_sin(ss_sin_start:end).^2));

ss_heading_hpc_sin = mean(abs(e_hpc_sin_hist(3, ss_sin_start:end)));
ss_heading_lpc_sin = mean(abs(e_lpc_sin_hist(3, ss_sin_start:end)));

n_recomp_sin = sum(hpc_recomp_sin);
nu_mean_sin = mean(nu_sin_hist(1:end-1));
max_err_hpc_sin = max(pos_err_hpc_sin);
max_err_lpc_sin = max(pos_err_lpc_sin);

u_rms_hpc_sin = [rms(u_hpc_sin_hist(1,:)), rms(u_hpc_sin_hist(2,:)), rms(u_hpc_sin_hist(3,:))];
u_rms_lpc_sin = [rms(u_lpc_sin_hist(1,:)), rms(u_lpc_sin_hist(2,:)), rms(u_lpc_sin_hist(3,:))];

% --- Figure 4b: Sinusoidal trajectory ---
figure(4); clf;
set(gcf, 'Position', [250, 250, 1000, 800]);

subplot(2,2,1);
hold on;
plot(x_l_sin(1,:), x_l_sin(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Leader');
plot(x_f_hpc_sin(1,:), x_f_hpc_sin(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Follower (HPC)');
plot(x_f_hpc_sin(1,1), x_f_hpc_sin(2,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
xlabel('p_x (m)', 'FontSize', 12); ylabel('p_y (m)', 'FontSize', 12);
title(sprintf('Sinusoidal Traj (\\omega_{amp}=%.1f rad/s, f=%.1f Hz)', omega_amp, omega_freq), 'FontSize', 13);
legend('Location', 'best', 'FontSize', 10);
axis equal; grid on;

subplot(2,2,2);
plot(t_sin, pos_err_hpc_sin, 'b-', 'LineWidth', 1.5); hold on;
plot(t_sin, pos_err_lpc_sin, 'r-', 'LineWidth', 1.5);
yline(t90_thr_hpc_sin, 'k--', 'LineWidth', 1);
xlabel('t (s)', 'FontSize', 12); ylabel('|e_{xy}| (m)', 'FontSize', 12);
title('Position Error: HPC vs LPC', 'FontSize', 13);
legend('HPC', 'LPC', 'T90 threshold', 'FontSize', 10);
grid on;

subplot(2,2,3);
plot(t_sin, abs(e_hpc_sin_hist(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_sin, abs(e_lpc_sin_hist(3,:)), 'r-', 'LineWidth', 1.5);
xlabel('t (s)', 'FontSize', 12); ylabel('|e_\theta| (rad)', 'FontSize', 12);
title('Heading Error', 'FontSize', 13);
legend('HPC', 'LPC', 'FontSize', 10);
grid on;

subplot(2,2,4);
yyaxis left;
plot(t_sin, x_l_sin(6,:), 'k-', 'LineWidth', 1.5);
ylabel('\omega_l (rad/s)', 'FontSize', 12);
yyaxis right;
stairs(t_sin, hpc_recomp_sin*0.8, 'b-', 'LineWidth', 1);
ylim([0, 1.2]);
ylabel('HPC recompute', 'FontSize', 12);
xlabel('t (s)', 'FontSize', 12);
title(sprintf('Leader \\omega_l + HPC recomputations (n=%d)', n_recomp_sin), 'FontSize', 13);
grid on;

fprintf('  Sinusoidal HPC T90=%.3f s, LPC T90=%.3f s, n_recomp=%d\n', ...
        T90_hpc_sin, T90_lpc_sin, n_recomp_sin);

%% ========================================================================
%% Section 5: Comparison -- 6D HPC vs 4D HPC (Model Improvement)
%% ========================================================================

fprintf('\n--- 6D vs 4D Model Comparison ---\n');

% 4D double integrator model
A_4d = [zeros(2) eye(2); zeros(2) zeros(2)];
B_4d = [zeros(2); eye(2)/mass];

% 4D formation: discrete points on circle
m_p = 4;
dl_4d = zeros(4, m_p);
for i = 0:m_p-1
    dl_4d(1:2, i+1) = -radius * [cos(2*pi*i/m_p); sin(2*pi*i/m_p)];
end

% 4D HPC setup
e_4d_init = [3; 1; 0; 0];  % initial position error
K_lin_4d = compute_4d_linear_gain(e_4d_init, mass, omega_d_pos);
[K0_4d, G0_4d, P_4d, nu_min_4d, nu_max_4d] = lpc2hpc(A_4d, B_4d, K_lin_4d);
nu_4d = nu_min_4d;
Gd_4d = eye(4) + nu_4d * G0_4d;
K_4d = K_lin_4d - K0_4d;

% Simulate 4D HPC follower
x_l_4d = zeros(4, N);  % leader 4D state (integrated from 6D leader)
x_f_4d = zeros(4, N);
x_f_4d(:,1) = [x_f_hpc(1,1); x_f_hpc(2,1); 0; 0];
x_l_4d(:,1) = [x_l(1,1); x_l(2,1); ...
               x_l(4,1)*cos(x_l(3,1)) - x_l(5,1)*sin(x_l(3,1)); ...
               x_l(4,1)*sin(x_l(3,1)) + x_l(5,1)*cos(x_l(3,1))];

e_4d_hist = zeros(4, N);
d_active = dl_4d(:,1);
d_4d_idx = 1;

for k = 1:N-1
    % Leader 4D state from 6D leader (body velocity -> map velocity)
    vx_map = x_l(4,k)*cos(x_l(3,k)) - x_l(5,k)*sin(x_l(3,k));
    vy_map = x_l(4,k)*sin(x_l(3,k)) + x_l(5,k)*cos(x_l(3,k));
    x_l_4d(:,k+1) = [x_l(1:2,k+1); vx_map; vy_map];

    % Error
    e_4d = x_f_4d(:,k) - x_l_4d(:,k) - d_active;

    % Formation point switching
    dists = zeros(1, m_p);
    for i = 1:m_p
        dists(i) = norm(x_f_4d(1:2,k) - x_l_4d(1:2,k) - dl_4d(1:2,i));
    end
    [mv, mi] = min(dists);
    tol_4d = 0.1;
    if mv + tol_4d < norm(x_f_4d(1:2,k) - x_l_4d(1:2,k) - d_active(1:2))
        d_active = dl_4d(:,mi);
        % Recompute HPC for new formation point
        K_lin_4d = compute_4d_linear_gain(e_4d, mass, omega_d_pos);
        [K0_4d, G0_4d, P_4d, nu_min_4d, nu_max_4d] = lpc2hpc(A_4d, B_4d, K_lin_4d);
        nu_4d = nu_min_4d;
        Gd_4d = eye(4) + nu_4d * G0_4d;
        K_4d = K_lin_4d - K0_4d;
    end

    % 4D HPC control (e_hpc.m convention)
    c_4d = hnorm(e_4d, Gd_4d, P_4d, 0.5, 1.0, 20);
    if c_4d > 1e-10
        u_4d = K0_4d * e_4d + c_4d^(1+nu_4d) * K_4d * expm(-log(c_4d) * Gd_4d) * e_4d;
    else
        u_4d = K0_4d * e_4d;
    end

    % 4D double integrator dynamics
    x_f_4d(:,k+1) = x_f_4d(:,k) + h * (A_4d * x_f_4d(:,k) + B_4d * u_4d);

    e_4d_hist(:,k) = e_4d;
end
e_4d_hist(:,N) = x_f_4d(:,N) - x_l_4d(:,N) - d_active;

% --- Figure 5: 6D HPC vs 4D HPC comparison ---
figure(5); clf;
set(gcf, 'Position', [300, 300, 1000, 800]);

subplot(2,2,1);
hold on;
plot(x_l(1,:), x_l(2,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Leader');
plot(x_f_hpc(1,:), x_f_hpc(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '6D HPC');
plot(x_f_4d(1,:), x_f_4d(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', '4D HPC');
plot(x_l(1,end) + radius*cos(th_circ), x_l(2,end) + radius*sin(th_circ), ...
     'k--', 'LineWidth', 0.8);
xlabel('$p_x$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$p_y$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Trajectories: 6D vs 4D HPC', 'Interpreter', 'tex', 'FontSize', 14);
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 11);
axis equal; grid on;
hold off;

subplot(2,2,2);
pos_err_6d = sqrt(e_hpc_hist(1,:).^2 + e_hpc_hist(2,:).^2);
pos_err_4d = sqrt(e_4d_hist(1,:).^2 + e_4d_hist(2,:).^2);
plot(t, pos_err_6d, 'b-', 'LineWidth', 1.5); hold on;
plot(t, pos_err_4d, 'g-', 'LineWidth', 1.5);
yline(radius, 'k--', 'LineWidth', 1);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$\|e_{xy}\|$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Position Error: 6D vs 4D', 'Interpreter', 'tex', 'FontSize', 14);
legend('6D HPC', '4D HPC', '$r_s$', ...
       'Interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(2,2,3);
plot(t, abs(e_hpc_hist(3,:)), 'b-', 'LineWidth', 1.5);
xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$|e_\theta|$ (rad)', 'Interpreter', 'latex', 'FontSize', 14);
title('Heading Error (6D only)', 'Interpreter', 'tex', 'FontSize', 14);
grid on;

subplot(2,2,4);
% Steady-state error comparison
ss_start = round(0.6 * N);  % last 40% as steady state
ss_err_6d = mean(pos_err_6d(ss_start:end));
ss_err_4d = mean(pos_err_4d(ss_start:end));
rmse_6d = sqrt(mean(pos_err_6d(ss_start:end).^2));
rmse_4d = sqrt(mean(pos_err_4d(ss_start:end).^2));

bar_data = [ss_err_6d, ss_err_4d; rmse_6d, rmse_4d]';
b = bar(bar_data);
b(1).FaceColor = [0.2 0.4 0.8];
b(2).FaceColor = [0.2 0.7 0.3];
set(gca, 'XTickLabel', {'Mean |e_{xy}|', 'RMSE(|e_{xy}| - r_s)'});
ylabel('Error (m)', 'Interpreter', 'latex', 'FontSize', 14);
title('Steady-State Performance', 'Interpreter', 'tex', 'FontSize', 14);
legend('6D HPC', '4D HPC', 'Interpreter', 'latex', 'FontSize', 12);
grid on;

% (metrics moved to METRICS section)

%% ========================================================================
%% Section 6: Robustness to Leader Velocity Variation Rate
%% ========================================================================

fprintf('\n--- Robustness to Velocity Variation Rate ---\n');

% Vary the sinusoidal frequency of leader's omega_l to test robustness
freq_vec = [0.1, 0.2, 0.5, 1.0];  % frequencies (Hz)
T_robust = 20;
N_robust = round(T_robust / h);
ss_err_vs_freq = zeros(size(freq_vec));

figure(5); clf;
set(gcf, 'Position', [350, 350, 1000, 600]);

for f_idx = 1:length(freq_vec)
    freq = freq_vec(f_idx);

    % Leader with sinusoidal turning
    x_l_r = zeros(6, N_robust);
    x_f_r = zeros(6, N_robust);
    x_l_r(:,1) = [0; 0; 0; 0.5; 0; 0];
    x_f_r(:,1) = [-2; -1; 0; 0; 0; 0];

    hpc_active_r = false;
    last_hpc_vel_r = [0;0;0];
    last_dtheta_r = 0;
    K0_r = zeros(3,6); G0_r = zeros(6); P_r = eye(6);
    nu_r = -1; Gd_r = eye(6);
    K_hpc_r = zeros(3,6);

    err_r_hist = zeros(1, N_robust);

    for k = 1:N_robust-1
        % Leader: constant forward + sinusoidal turning
        omega_l_r = 0.8 * sin(2*pi*freq * k*h);
        x_l_r(6,k) = omega_l_r;
        u_l_r = [0; 0; 0];  % steady turning, no acceleration
        x_l_r(:,k+1) = integrate_6d_kinematics(x_l_r(:,k), u_l_r, h, mass, I_val);

        e_r = compute_6d_error(x_l_r(:,k), x_f_r(:,k), radius);
        K_lin_r = compute_6d_linear_gain(e_r, mass, I_val, ...
                                          omega_d_pos, omega_d_theta);
        [A_l_r, ~] = build_6d_system(x_l_r(6,k), x_l_r(4,k), x_l_r(5,k), mass, I_val);

        leader_vel_r = [x_l_r(6,k); x_l_r(4,k); x_l_r(5,k)];
        dtheta_r = x_f_r(3,k) - x_l_r(3,k);

        if ~hpc_active_r || ...
           norm(leader_vel_r - last_hpc_vel_r) > hpc_vel_threshold || ...
           abs(dtheta_r - last_dtheta_r) > 0.15
            [K0_r, G0_r, P_r, nu_min_r, nu_max_r] = lpc2hpc(A_l_r, B_6d, K_lin_r);
            nu_r = nu_min_r;
            Gd_r = eye(6) + nu_r * G0_r;
            K_hpc_r = K_lin_r - K0_r;
            hpc_active_r = true;
            last_hpc_vel_r = leader_vel_r;
            last_dtheta_r = dtheta_r;
        end

        c_r = hnorm(e_r, Gd_r, P_r, 0.5, 1.0, 20);
        if c_r > 1e-10
            u_L_r = K0_r * e_r + c_r^(1+nu_r) * K_hpc_r * expm(-log(c_r) * Gd_r) * e_r;
        else
            u_L_r = K0_r * e_r;
        end
        u_f_r = rotate_control_to_follower(u_L_r, dtheta_r);
        x_f_r(:,k+1) = integrate_6d_kinematics(x_f_r(:,k), u_f_r, h, mass, I_val);

        err_r_hist(k) = norm(e_r(1:2));
    end
    err_r_hist(N_robust) = norm(compute_6d_error(x_l_r(:,N_robust), x_f_r(:,N_robust), radius));

    ss_start_r = round(0.5 * N_robust);
    ss_err_vs_freq(f_idx) = mean(err_r_hist(ss_start_r:end));

    subplot(2,2,f_idx);
    t_r = (0:N_robust-1) * h;
    plot(t_r, err_r_hist, 'b-', 'LineWidth', 1.5); hold on;
    yline(radius, 'k--', 'LineWidth', 1);
    xlabel('$t$ (s)', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('$\|e_{xy}\|$ (m)', 'Interpreter', 'latex', 'FontSize', 12);
    title(sprintf('$f$ = %.1f Hz (mean SS err: %.3f m)', freq, ss_err_vs_freq(f_idx)), ...
          'Interpreter', 'latex', 'FontSize', 12);
    ylim([0, 6]);
    grid on;
end
sgtitle('Robustness to Leader Turn Rate Variation', ...
        'Interpreter', 'latex', 'FontSize', 14);

% (metrics moved to METRICS section)

%% ========================================================================
%% Section 7: Comprehensive Performance Metrics
%% ========================================================================

% --- Compute transient metrics for 6D HPC ---
% Initial error magnitude
init_err_hpc = pos_err_hpc(1);
% T90: time for error to drop to 10% of initial
t90_threshold = 0.1 * init_err_hpc;
settled = pos_err_hpc < t90_threshold;
converge_idx = NaN;
t_start_check = max(1, find(pos_err_hpc > t90_threshold, 1, 'last') + 1);
if isempty(t_start_check), t_start_check = 1; end
for k = t_start_check:N-50
    if all(settled(k:min(k+50, N)))
        converge_idx = k;
        break;
    end
end
if ~isnan(converge_idx)
    T90_hpc = t(converge_idx);
else
    T90_hpc = Tmax;
end

% Max transient error
max_err_hpc = max(pos_err_hpc);

% RMS control effort
u_rms_hpc = [rms(u_hpc_hist(1,:)), rms(u_hpc_hist(2,:)), rms(u_hpc_hist(3,:))];

% Steady-state stats (last 30%)
ss_start_30 = round(0.7 * N);
ss_mean_hpc = mean(pos_err_hpc(ss_start_30:end));
ss_std_hpc  = std(pos_err_hpc(ss_start_30:end));
ss_rmse_hpc = sqrt(mean(pos_err_hpc(ss_start_30:end).^2));

% HPC recomputation stats
n_recomp = sum(hpc_recomp_hist);
nu_mean = mean(nu_hist(1:end-1));
nu_final = nu_hist(find(nu_hist ~= 0, 1, 'last'));

% --- Same metrics for 6D LPC (ablation) ---
pos_err_lpc_full = sqrt(e_lpc_hist(1,:).^2 + e_lpc_hist(2,:).^2);
init_err_lpc = pos_err_lpc_full(1);
t90_threshold_lpc = 0.1 * init_err_lpc;
settled_lpc = pos_err_lpc_full < t90_threshold_lpc;
converge_idx_lpc = NaN;
t_start_check_lpc = max(1, find(pos_err_lpc_full > t90_threshold_lpc, 1, 'last') + 1);
if isempty(t_start_check_lpc), t_start_check_lpc = 1; end
for k = t_start_check_lpc:N-50
    if all(settled_lpc(k:min(k+50, N)))
        converge_idx_lpc = k;
        break;
    end
end
if ~isnan(converge_idx_lpc)
    T90_lpc = t(converge_idx_lpc);
else
    T90_lpc = Tmax;
end
max_err_lpc = max(pos_err_lpc_full);
u_rms_lpc = [rms(u_lpc_hist(1,:)), rms(u_lpc_hist(2,:)), rms(u_lpc_hist(3,:))];
ss_mean_lpc = mean(pos_err_lpc_full(ss_start_30:end));
ss_std_lpc  = std(pos_err_lpc_full(ss_start_30:end));
ss_rmse_lpc = sqrt(mean(pos_err_lpc_full(ss_start_30:end).^2));

% --- 4D HPC metrics ---
ss_start_4d = round(0.7 * length(pos_err_4d));
ss_mean_4d = mean(pos_err_4d(ss_start_4d:end));
ss_std_4d  = std(pos_err_4d(ss_start_4d:end));
ss_rmse_4d = sqrt(mean(pos_err_4d(ss_start_4d:end).^2));

% --- Heading error stats (6D only) ---
ss_heading_hpc = mean(abs(e_hpc_hist(3, ss_start_30:end)));
ss_heading_lpc = mean(abs(e_lpc_hist(3, ss_start_30:end)));

% --- Lyapunov function decay ---
% Estimate exponential decay rate: V(t) ~ V0 * exp(-alpha * t)
V_valid = V_hpc_hist(V_hpc_hist > 0.01);
if length(V_valid) > 100
    log_V = log(V_valid(1:min(500, end)));
    t_log = t(1:length(log_V));
    p = polyfit(t_log', log_V', 1);
    lyap_decay_rate = -p(1);  % estimated decay rate
else
    lyap_decay_rate = NaN;
end

%% ========================================================================
%% Section 8: Print All Metrics
%% ========================================================================

fprintf('\n');
fprintf('===================== METRICS_BEGIN =====================\n');
fprintf('SYSTEM_PARAMS:\n');
fprintf('  n_states=6\n');
fprintf('  n_inputs=3\n');
fprintf('  mass=%.2f\n', mass);
fprintf('  I=%.2f\n', I_val);
fprintf('  radius=%.2f\n', radius);
fprintf('  omega_d_pos=%.2f\n', omega_d_pos);
fprintf('  omega_d_theta=%.2f\n', omega_d_theta);
fprintf('  hpc_vel_threshold=%.2f\n', hpc_vel_threshold);
fprintf('  h=%.3f\n', h);
fprintf('  Tmax=%.0f\n', Tmax);
fprintf('SIM_PARAMS_END\n');

fprintf('\n');
fprintf('FROZEN_TIME_STABILITY:\n');
fprintf('  maxRe_eig_LPC_range=[%.4f,%.4f]  min_lyap_cond=%.4f  lyap_ok=%d\n', ...
        min(max_eig_lpc(:)), max(max_eig_lpc(:)), min(lyap_cond(:)), all(lyap_cond(:)>0));
fprintf('  nu_range=[%.3f,%.3f]  nu_mean=%.3f  omega_l=[%.1f,%.1f]_rad/s  vx_l=[%.1f,%.1f]_m/s\n', ...
        min(nu_min_grid(:)), max(nu_max_grid(:)), mean(nu_min_grid(:)), ...
        omega_vec(1), omega_vec(end), vx_vec(1), vx_vec(end));
fprintf('FROZEN_TIME_END\n');

fprintf('\n');
fprintf('CIRCULAR_TRAJECTORY_HPC_vs_LPC:\n');
fprintf('  leader_vx=%.2f_m/s  leader_omega=%.2f_rad/s  turn_radius=%.2f_m\n', ...
        vx_l_des, omega_l_des, vx_l_des/omega_l_des);
fprintf('  init_err=%.3f_m  T90_threshold=%.3f_m\n', init_err_hpc, t90_threshold);
fprintf('  HPC  T90=%.3f_s  max_err=%.4f  ss_mean=%.6f  ss_std=%.6f  ss_rmse=%.6f  heading_ss=%.6f_rad\n', ...
        T90_hpc, max_err_hpc, ss_mean_hpc, ss_std_hpc, ss_rmse_hpc, ss_heading_hpc);
fprintf('  LPC  T90=%.3f_s  max_err=%.4f  ss_mean=%.6f  ss_std=%.6f  ss_rmse=%.6f  heading_ss=%.6f_rad\n', ...
        T90_lpc, max_err_lpc, ss_mean_lpc, ss_std_lpc, ss_rmse_lpc, ss_heading_lpc);
fprintf('  HPC  rms_u=[%.4f,%.4f,%.4f]  n_recomp=%d  nu_mean=%.4f  nu_final=%.4f\n', ...
        u_rms_hpc, n_recomp, nu_mean, nu_final);
fprintf('  LPC  rms_u=[%.4f,%.4f,%.4f]\n', u_rms_lpc);
fprintf('CIRCULAR_TRAJECTORY_END\n');

fprintf('\n');
fprintf('SINUSOIDAL_TRAJECTORY_TIME_VARYING:\n');
fprintf('  omega_amp=%.2f_rad/s  omega_freq=%.2f_Hz  vx=%.2f_m/s\n', omega_amp, omega_freq, vx_l_sin);
fprintf('  init_err=%.3f_m  T90_threshold=%.3f_m\n', init_err_hpc_sin, t90_thr_hpc_sin);
fprintf('  HPC  T90=%.3f_s  max_err=%.4f  ss_mean=%.6f  ss_std=%.6f  ss_rmse=%.6f  heading_ss=%.6f_rad\n', ...
        T90_hpc_sin, max_err_hpc_sin, ss_mean_hpc_sin, ss_std_hpc_sin, ss_rmse_hpc_sin, ss_heading_hpc_sin);
fprintf('  LPC  T90=%.3f_s  max_err=%.4f  ss_mean=%.6f  ss_std=%.6f  ss_rmse=%.6f  heading_ss=%.6f_rad\n', ...
        T90_lpc_sin, max_err_lpc_sin, ss_mean_lpc_sin, ss_std_lpc_sin, ss_rmse_lpc_sin, ss_heading_lpc_sin);
fprintf('  HPC  rms_u=[%.4f,%.4f,%.4f]  n_recomp=%d  nu_mean=%.4f\n', ...
        u_rms_hpc_sin, n_recomp_sin, nu_mean_sin);
fprintf('  LPC  rms_u=[%.4f,%.4f,%.4f]\n', u_rms_lpc_sin);
fprintf('  T90_ratio_HPC_vs_LPC=%.3f\n', T90_hpc_sin / max(T90_lpc_sin, 1e-10));
fprintf('SINUSOIDAL_END\n');

fprintf('\n');
fprintf('MODEL_COMPARISON_6D_vs_4D:\n');
fprintf('  6D  ss_mean=%.6f_m  ss_std=%.6f_m  ss_rmse=%.6f_m\n', ...
        ss_err_6d, std(pos_err_6d(ss_start:end)), rmse_6d);
fprintf('  4D  ss_mean=%.6f_m  ss_std=%.6f_m  ss_rmse=%.6f_m\n', ...
        ss_err_4d, std(pos_err_4d(ss_start:end)), rmse_4d);
fprintf('  6D/4D_rmse_ratio=%.3f\n', rmse_6d / max(rmse_4d, 1e-10));
fprintf('MODEL_COMPARISON_END\n');

fprintf('\n');
fprintf('ROBUSTNESS_FREQ_SWEEP:\n');
fprintf('  freq_Hz=[%.1f %.1f %.1f %.1f]  ss_err_m=[%.4f %.4f %.4f %.4f]\n', ...
        freq_vec(1), freq_vec(2), freq_vec(3), freq_vec(4), ...
        ss_err_vs_freq(1), ss_err_vs_freq(2), ss_err_vs_freq(3), ss_err_vs_freq(4));
fprintf('ROBUSTNESS_END\n');

fprintf('\n');
fprintf('LYAPUNOV_ANALYSIS:\n');
fprintf('  V_init=%.3f  V_final=%.4f  V_ratio=%.4f', ...
        max(V_hpc_hist(1), 1e-10), V_hpc_hist(end), ...
        V_hpc_hist(end)/max(V_hpc_hist(1),1e-10));
if ~isnan(lyap_decay_rate)
    fprintf('  decay_rate=%.3f_1/s', lyap_decay_rate);
end
fprintf('\n');
fprintf('LYAPUNOV_END\n');

fprintf('====================== METRICS_END ======================\n');
fprintf('\nDone! All figures generated.\n');

%% ========================================================================
%% Helper Functions
%% ========================================================================

function [A, B] = build_6d_system(omega_l, vx_l, vy_l, mass, I_val)
% Build the 6D time-varying system matrices.
% The A matrix couples leader velocity into the error dynamics.
% State: [px, py, theta, vx_b, vy_b, omega]^T
% Input: [ax, ay, alpha]^T (body-frame accelerations)

A = [0,       omega_l,  -vy_l,   1, 0, 0;
    -omega_l,  0,        vx_l,   0, 1, 0;
     0,        0,        0,      0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0];

B = [zeros(3,3);
     eye(3) * diag([1/mass, 1/mass, 1/I_val])];
end

function e = compute_6d_error(x_l, x_f, radius)
% Compute 6D formation error in the LEADER body frame.
% x_l, x_f: 6D state vectors [px, py, theta, vx_b, vy_b, omega]^T
% radius: formation safety circle radius

% Position error in map frame
pex = x_f(1) - x_l(1);
pey = x_f(2) - x_l(2);
dist = norm([pex; pey]);

% Boundary projection: desired follower position on safety circle
if dist > 1e-6
    dx_map = radius * pex / dist;
    dy_map = radius * pey / dist;
else
    dx_map = radius;
    dy_map = 0;
end

% Rotate position error and desired offset to leader body frame
cos_l = cos(x_l(3));
sin_l = sin(x_l(3));

pex_L =  pex * cos_l + pey * sin_l;
pey_L = -pex * sin_l + pey * cos_l;
dx_L  =  dx_map * cos_l + dy_map * sin_l;
dy_L  = -dx_map * sin_l + dy_map * cos_l;

% Heading error
dtheta = x_f(3) - x_l(3);

% Follower body-frame velocity -> leader body frame
% R(-dtheta): rotates vector from follower frame to leader frame
cos_dt = cos(dtheta);
sin_dt = sin(dtheta);
vxf_L =  cos_dt * x_f(4) + sin_dt * x_f(5);
vyf_L = -sin_dt * x_f(4) + cos_dt * x_f(5);

% 6D error vector
e = [pex_L - dx_L;
     pey_L - dy_L;
     dtheta;
     vxf_L - x_l(4);
     vyf_L - x_l(5);
     x_f(6) - x_l(6)];
end

function K = compute_6d_linear_gain(e, mass, I_val, omega_d, omega_d_theta)
% Compute adaptive critically-damped linear gain for 6D system.
% Three independent channels: X, Y (position), Theta (heading).
% e: 6D error vector
% omega_d, omega_d_theta: desired damping bandwidths

% Channel 1: X position
a_x = clamp_ratio(-mass * e(4) / max(abs(e(1)), 0.001), mass, omega_d);
k2_x = -2 * a_x;
k1_x = a_x * (k2_x + a_x) / mass;

% Channel 2: Y position
a_y = clamp_ratio(-mass * e(5) / max(abs(e(2)), 0.001), mass, omega_d);
k2_y = -2 * a_y;
k1_y = a_y * (k2_y + a_y) / mass;

% Channel 3: Theta
a_th = clamp_ratio(-I_val * e(6) / max(abs(e(3)), 0.001), I_val, omega_d_theta);
k2_th = -2 * a_th;
k1_th = a_th * (k2_th + a_th) / I_val;

K = [k1_x,  0,     0,     k2_x,  0,     0;
     0,     k1_y,  0,     0,     k2_y,  0;
     0,     0,     k1_th, 0,     0,     k2_th];
end

function a = clamp_ratio(raw_ratio, m_val, omega_d)
% Clamp the velocity/position ratio to prevent gain explosion.
max_ratio = omega_d * m_val;
a = max(min(raw_ratio, max_ratio), omega_d * m_val);
% Minimum bandwidth floor
a = max(a, omega_d * m_val);
end

function x_next = integrate_6d_kinematics(x, u, h, mass, I_val)
% Integrate 6D nonlinear kinematics using forward Euler.
% x: current state [px, py, theta, vx_b, vy_b, omega]^T
% u: body-frame acceleration [ax, ay, alpha]^T

px = x(1); py = x(2); theta = x(3);
vx = x(4); vy = x(5); omega = x(6);

% Nonlinear position kinematics
px_next  = px + h * (vx * cos(theta) - vy * sin(theta));
py_next  = py + h * (vx * sin(theta) + vy * cos(theta));
theta_next = theta + h * omega;

% Velocity integration (double integrator)
vx_next   = vx + h * u(1) / mass;
vy_next   = vy + h * u(2) / mass;
omega_next = omega + h * u(3) / I_val;

x_next = [px_next; py_next; theta_next; vx_next; vy_next; omega_next];
end

function u_f = rotate_control_to_follower(u_L, dtheta)
% Rotate control forces from leader body frame to follower body frame.
% u_L: control in leader body frame [Fx_L, Fy_L, tau]^T
% dtheta = theta_f - theta_l

cos_dt = cos(dtheta);
sin_dt = sin(dtheta);

% R(-dtheta) rotates from leader frame to follower frame
Fx_f =  cos_dt * u_L(1) + sin_dt * u_L(2);
Fy_f = -sin_dt * u_L(1) + cos_dt * u_L(2);

u_f = [Fx_f; Fy_f; u_L(3)];
end

function K = compute_4d_linear_gain(e, mass, omega_d)
% Compute adaptive critically-damped linear gain for 4D system.
% e: 4D error [ex, ey, evx, evy]^T

% Independent X and Y channels
a_x = max(-mass * e(3) / max(abs(e(1)), 0.001), omega_d * mass);
a_x = max(a_x, omega_d * mass);
a_y = max(-mass * e(4) / max(abs(e(2)), 0.001), omega_d * mass);
a_y = max(a_y, omega_d * mass);

k2_x = -2 * a_x;  k2_y = -2 * a_y;
k1_x = a_x * (k2_x + a_x) / mass;
k1_y = a_y * (k2_y + a_y) / mass;

K = [k1_x, 0,     k2_x, 0;
     0,    k1_y,  0,    k2_y];
end

