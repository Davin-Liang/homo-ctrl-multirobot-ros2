%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Example of upgrading a linear control to HPC and FHPC 
%%   
%%  System:      dx/dt=A*x+B*u  
%%                
%%        where  
%%               x - system  state vector (n x 1)
%%               u - control input (m x 1)
%%               A - system matrix (n x n)
%%               B - control matrix (m x m)
%%      
%% System for an upgrade is a linearized model of rotary inverted pendulum 
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pendulum model 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Motor
% % Resistance
% Rm = 8.4;
% % Current-torque (N-m/A)
% kt = 0.042;
% % Back-emf constant (V-s/rad)
% km = 0.042;
% %
% %% Rotary Arm
% % Mass (kg)
% mr = 0.095;
% % Total length (m)
% r = 0.085;
% % Moment of inertia about pivot (kg-m^2)
% Jr = mr*r^2/3;
% % Equivalent Viscous Damping Coefficient (N-m-s/rad)
% br = 1e-3; % damping tuned heuristically to match QUBE-Sero 2 response
% %
% %% Pendulum Link
% % Mass (kg)
% mp = 0.024;
% % Total length (m)
% Lp = 0.129;
% % Pendulum center of mass (m)
% l = Lp/2;
% % Moment of inertia about pivot (kg-m^2)
% Jp = mp*Lp^2/3;
% % Equivalent Viscous Damping Coefficient (N-m-s/rad)
% bp = 5e-5; % damping tuned heuristically to match QUBE-Sero 2 response
% % Gravity Constant
% g = 9.81;
% 
% % Total Inertia
% Jt = Jr*Jp - mp^2*r^2*l^2;
% % 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Linearized model of the rotary inverted pendulum in the upper position
%%
%%           dx/dt=Ax+Bu,      x=(x1,x2,x3,x4)'
%%
%% where     x1  - angle of the pentulum arm
%%           x2  - angle of the rotary arm
%%           x3  - angular velocity of the pendulum arm
%%           x4  - angular velocity of the rotary arm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A = [0 0 1 0;
%      0 0 0 1;
%      0 mp^2*l^2*r*g/Jt  -br*Jp/Jt   -mp*l*r*bp/Jt 
%      0  mp*g*l*Jr/Jt    -mp*l*r*br/Jt   -Jr*bp/Jt];
% %
% B = [0; 0; Jp/Jt; mp*l*r/Jt];
% 
% % adding a model of actuator dynamics
% A(3,3) = A(3,3) - km*km/Rm*B(3);
% A(4,3) = A(4,3) - km*km/Rm*B(4);
% B = km * B / Rm;
% 
%  % the linear feedback gain (provided by manufacturer)
% Klin=[2 -35 1.5 -3];
m=2;%mass
A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)*1/m];

lambda = 2;
k2 = -3;
k1 = lambda*(k2 + lambda)/m;
k_lin=[eye(2)*k1 eye(2)*k2];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HPC/FHPC design by upgrading a linear controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[K0 G0 P mu_min mu_max]=lpc2hpc(A,B,k_lin); % upgrade linear control to HPC

%selection of the homogeneity degree mu_min<= mu <=mu_max

Gd=eye(4)+mu_min*G0;  mu=mu_min; % for HCP with negative homogeneity degree
%Gd=eye(4)+mu_max*G0; mu=mu_max; % for HCP with positive homogeneity degree
                                 % for FHCP use G0 mu_min mu_max 
K=k_lin-K0;   

%K0 - homogenization feedback gain
%K  - control gain
%Gd - generator of dilation
%P  - shape matrix of the weighted Euclidean norm


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Numerical Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t=0; Tmax=16; 
h=0.001; % sampling period

r1=[-sin(t)/m;-cos(t)/m];
x1=[r1;[-cos(t)/m;sin(t)/m]];
x2=[[5;2];[3;2]];

% x=[1;1;0;0];
% tl=[t];xl=[x];ul=[];

tl=[t];xl1=[x1];ul1=[];
xl2=[x2];ul2=[];

%alpha and beta are tuning parameters (alpha=beta=1 => linear control)
alpha=0.1; beta=1;  %for HPC with negative degree (upgrade close to zero)
%alpha=1;beta=100;   %for HPC with positive degree (upgrade close to Inf)                    
%alpha=0.1;beta=100; %for FHPC               

noise=0; %magnitude of measurement noises (may be changed for comparison)

disp('Run numerical simulation...');

[Ah Bh]=ZOH(h,A,B); %discretization of linear plant by ZOH

while t<Tmax
%     xm=x+2*noise*(rand(4,1)-0.5);          %modeling of noised measurement
    xm=x2;
    %u=Klin*xm;                            %linear control (for comparison)
    
    u=e_hpc(xm,K0,K,Gd,P,mu,alpha,beta);               %explicit HPC

    u1=[sin(t);cos(t)];
    x1=x1+h*(A*x1+B*u1); 

    u2=u+u1;
%     x2=x2++h*(A*x2+B*u2);
    
    %u=e_fhpc(xm,K0,K,G0,P,mu_min,mu_max,alpha,beta);  %explicit FHPC
    
    %simulation of the system (with control saturation as in QUBE Servo-2) 

    x2=Ah*x2+Bh*min(10,max(-10,u2));
%     x2=Ah*x2+Bh*u2;

    t=t+h;  tl=[tl t]; %xl=[xl x]; ul=[ul u];
    xl1=[xl1 x1];
    ul1=[ul1 u1];
    xl2=[xl2 x2];
    ul2=[ul2 u2];
end;
% ul=[ul u];

disp('Done!');

%%norm of the state at the time instant Tmax
disp(['||x(Tmax)||=',num2str(norm(x2))])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot simulation results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% figure;
% axes1 = subplot(1,2,1);
% hold(axes1,'on');
% plot1 = plot(tl,xl,'LineWidth',2,'Parent',axes1);
% set(plot1(1),'DisplayName','$x_1$');
% set(plot1(2),'DisplayName','$x_2$');
% set(plot1(3),'DisplayName','$x_3$');
% set(plot1(4),'DisplayName','$x_4$');
% ylabel('$x$','Interpreter','latex');
% xlabel('$t$','Interpreter','latex');
% title({'n=4'});
% xlim(axes1,[0 Tmax]);
% ylim(axes1,[-5 5]);
% box(axes1,'on');
% hold(axes1,'off');
% set(axes1,'FontSize',30,'XGrid','on','YGrid','on');
% legend1 = legend(axes1,'show');
% set(legend1,'Interpreter','latex');
% 
% 
% axes2 = subplot(1,2,2);
% hold(axes2,'on');
% plot2 = plot(tl,ul,'LineWidth',2);
% ylabel('$u$','Interpreter','latex');
% xlabel('$t$','Interpreter','latex');
% title({'HPC, m=1'});
% xlim(axes2,[0 Tmax]);
% ylim(axes2,[-5 5]);
% box(axes2,'on');
% hold(axes2,'off');
% set(axes2,'FontSize',30,'XGrid','on','YGrid','on');

figure(1);
hold on;

% 绘制第一行作为曲线
plot1=plot(tl, xl1(1,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$x_1$');
plot2=plot(tl, xl2(1,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$x_2$');
legend('x_1','x_2');
grid on;

figure(2);
hold on;

% 绘制第一行作为曲线
plot1=plot(tl, xl1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$y_1$');
plot2=plot(tl, xl2(2,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$y_2$');
legend('y_1','y_2');
grid on;


figure(3);
hold on;

% 绘制曲线
plot1=plot(xl1(1,:), xl1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', 'xl1');
plot2 = plot(xl2(1,:), xl2(2,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', 'xl2');
legend('xl1','xl2');
% 设置图形属性
grid on;
xlabel('X');
ylabel('Y');
title('Time-Varying Curve');

