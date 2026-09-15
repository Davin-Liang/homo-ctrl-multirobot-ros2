t=0; Tmax=30; 
h=0.01; % sampling period
m=2;%mass
A=[zeros(2) eye(2); zeros(2) zeros(2)];
B=[zeros(2); eye(2)]*1/m;
x11=[1;0;0;0];
x22=[5;1;0;0];

m_p=6; radius=1;
tol=0.1;
dl=[];
for i=0:m_p-1
 d=-radius*[cos(2*pi*i/m_p); sin(2*pi*i/m_p)];
 dl=[dl [d;0;0]];
end
dist_l=[];
for i=1:m_p  
    dist_l=[dist_l norm(x22-x11-dl(:,i))];
end;
[mv mi]=min(dist_l);
d=dl(:,mi);

e_lin=x22-x11-d;

a=max(-m*(e_lin(3))/e_lin(1),1);
b=max(-m*(e_lin(4))/e_lin(2),1);
lambda =diag([a b]);
% k2=-2*lambda;
k2=-2.4*lambda;
% k2=-4*lambda;
k1=lambda*(k2+lambda)/m;


k_lin=[k1/2 k2/2];
[K0,G0,P,nu_min,nu_max]=lpc2hpc(A,B,k_lin);
K=k_lin-K0;
nu=nu_min;
% nu=-0.95;
Gd=eye(4)+nu*G0;

tl=[];xll1=[];ull1=[];
xll2=[];ull2=[];
ell=[];
hll=[];
mll=[];gll=[];
while t<Tmax
  
    
%     u1=-[eye(2) eye(2)]*x1+1*[sin(t);cos(t)];
    u11=-[eye(2) eye(2)]*x11+1*[cos(t);sin(t)];
    x11=x11+h*(A*x11+B*u11); 
    
    e_lin=x22-x11-d;
    g=0.5*([eye(2) eye(2)]*x11-1*[cos(t);sin(t)]);
%     nx=hnorm(e,Gd,P);
    %%     nx = max(alpha, min(beta,nx)) 
%     u2=2*max(min(1,nx),0.05)^(1+nu)*K*expm(Gd*(1-log(max(min(1,nx),0.05))))*e+u1; 
%     u2=2*max(min(1,nx),0.01)^(1+nu)*K*expm(Gd*(1-log(max(min(1,nx),0.01))))*e; 
%     u2=e_hpc(e, K0, K, Gd, P, nu, 0, 1);
    u22=2*k_lin*e_lin+u11; 
%     u2=nx^(1+nu)*K*expm(-log(nx)*Gd)*e;
    x22=x22+h*(A*x22+B*u22);

    dist_l=[];
    for i=1:m_p  
        dist_l=[dist_l norm(x22-x11-dl(:,i))];
    end
    [mv mi]=min(dist_l);


    if mv+tol<norm(x22-x11-d)
        d=dl(:,mi);
        e_lin=x22-x11-d;
        a=max(-m*(e_lin(3))/e_lin(1),1);
        b=max(-m*(e_lin(4))/e_lin(2),1);
        lambda =diag([a b]);
        k2=-2*lambda;
        k1=lambda*(k2+lambda)/m;
        k_lin=[k1 k2];
        [K0,G0,P,nu_min,nu_max]=lpc2hpc(A,B,k_lin);
        nu=nu_min;
        Gd=eye(4)+nu*G0;
    end
   
    t=t+h;
    tl=[tl t];
    xll1=[xll1 x11];
    ull1=[ull1 u11];
    xll2=[xll2 x22];
    ull2=[ull2 u22];
    ell=[ell e_lin];
    mll=[mll mi];
%     hll=[hll nx];
    gll=[gll g];

    %%%%%%%%switching rule
%  find di
%  
%     if  '?(conditioon  for switching)?' distance to di+ const < distance to d
%            K_lin=?????
%            d=???? 
%            end;0
    El=[];

end

    for i=1:3000
        El=[El sqrt(ell(1,i)^2+ell(2,i)^2)];
    end

figure(1);
hold on;

% 绘制第一行作为曲线
plot1=plot(tl, xll1(1,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$x_1$');
plot2=plot(tl, xll2(1,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$x_2$');
xlim([0 30])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$x$','FontSize', 20,'Interpreter','latex')
legend1=legend('$x_1$','$x_2$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;





figure(2);
hold on;

plot1=plot(tl, xll1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$y_1$');
plot2=plot(tl, xll2(2,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$y_2$');
xlim([0 30])
ylim([-1 1])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$y$','FontSize', 20,'Interpreter','latex')
legend1=legend('$y_1$','$y_2$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;

figure(3);
hold on;


% 绘制曲线
plot1=plot(xll1(1,:), xll1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$r_1$');
plot2 = plot(xll2(1,:), xll2(2,:), 'k','LineWidth', 2);
set(plot2, 'DisplayName', '$r_2$');
ylim([-1 2])
xlabel('$x$','FontSize', 14,'Interpreter','latex')
ylabel('$y$','FontSize', 14,'Interpreter','latex')
legend('xl1','xl2');
legend1=legend('leader robot','follower with linear controller','FontSize', 14,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;



figure(4);
hold on;

plot3=plot(tl, ull2(1,:), 'r','LineWidth', 2);
ylim([-12 6])
set(plot3, 'DisplayName', '$u_x$');
plot3=plot(tl, ull2(2,:), 'b','LineWidth', 2);
ylim([-12 2])
set(plot3, 'DisplayName', '$u_x$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$u$','FontSize', 20,'Interpreter','latex')
legend1=legend('$u_x$','$u_y$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;

% figure(4);
% hold on;
% 
% plot3=plot(tl, ul1(1,:), 'r','LineWidth', 2);
% ylim([-12 6])
% set(plot3, 'DisplayName', '$u_x$');
% plot3=plot(tl, ul1(2,:), 'b','LineWidth', 2);
% ylim([-12 2])
% set(plot3, 'DisplayName', '$u_x$');
% xlabel('$t$','FontSize', 20,'Interpreter','latex')
% ylabel('$u$','FontSize', 20,'Interpreter','latex')
% legend1=legend('$u_x$','$u_y$','FontSize', 20,'Box', 'off');
% set(legend1,'Interpreter','latex');
% grid on;

figure(5);
hold on;

plot4=plot(tl, abs(ell(1,:)), 'r','LineWidth', 2);
set(plot4, 'DisplayName', '$e_x$');
plot5=plot(tl, abs(ell(2,:)), 'b','LineWidth', 2);
set(plot5, 'DisplayName', '$e_y$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$\ell$','FontSize', 20,'Interpreter','latex')
legend1=legend('$\ell_x$','$\ell_y$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;


% figure(6)
% hold on;
% 
% plot3=plot(tl, U(1,:), 'r','LineWidth', 2);
% % ylim([0 50])
% xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
% ylabel('$u$','FontSize', 20,'Interpreter','latex')
% set(plot3, 'DisplayName', '$u_{lin}$');
% legend1=legend('$u_{hom}$','FontSize', 20,'Box', 'off');
% set(legend1,'Interpreter','latex');
% grid on;

figure(7)
hold on;

plot3=plot(tl, El(1,:), 'r','LineWidth', 2);
ylim([0 3.5])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$e$','FontSize', 20,'Interpreter','latex')
set(plot3, 'DisplayName', '$e_{lin}$');
legend1=legend('$e_{lin}$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;
