t=0; Tmax=30; 
h=0.01; % sampling period
m=2;%mass
A=[zeros(2) eye(2); zeros(2) zeros(2)];
B=[zeros(2); eye(2)*1/m];
x1=[1;0;0;0];
x2=[5;1;0;0];

m_p=4; radius=1;
tol=0.1;
dl=[];
for i=0:m_p-1
 d=-1*radius*[cos(2*pi*i/m_p); sin(2*pi*i/m_p)];
 dl=[dl [d;0;0]];
end
dist_l=[];
for i=1:m_p  
    dist_l=[dist_l norm(x2-x1-dl(:,i))];
end;
[mv mi]=min(dist_l);
d=dl(:,mi);

e=x2-x1-d;

a=max(-m*(e(3))/e(1),1);
b=max(-m*(e(4))/e(2),1);
% a=min(-m*(e(3))/e(1),1);
% b=min(-m*(e(4))/e(2),1);
% a=2;
% b=2;
lambda =diag([a b]);
k2=-2*lambda;
k1=lambda*(k2+lambda)/m;


k_lin=[k1 k2];
[K0,G0,P,nu_min,nu_max]=lpc2hpc(A,B,k_lin);
nu=nu_min;
% nu=0;
Gd=eye(4)+nu*G0;


tl=[];xl1=[];ul1=[];
xl2=[];ul2=[];
el=[];
lambdal=[];
ul=[];
u1l=[];
while t<Tmax
  
    
    u1=-[eye(2) eye(2)]*x1+1*[sin(t);cos(t)];
%     u1=-[eye(2) eye(2)]*x1+1*[cos(t);sin(t)];
    x1=x1+h*(A*x1+B*u1); 
    e=x2-x1-d;
    nx=hnorm(e,Gd,P);
    u2=max(min(1,nx),0.1)^(1+nu)*k_lin*expm(Gd*(1-log(max(min(1,nx),0.1))))*e; 
%     u2=max(min(0.02,nx),0.01)^(1+nu)*k_lin*expm(Gd*(1-log(max(min(0.02,nx),0.01))))*e;
%     u2=k_lin*e;
%     u2=min(0.1,nx)^(1+nu)*k_lin*expm(-Gd*log(min(0.1,nx)))*e; 

    x2=x2+h*(A*x2+B*u2);
%     u=u2-u1;

    dist_l=[];
    for i=1:m_p  
        dist_l=[dist_l norm(x2-x1-dl(:,i))];
    end
    [mv mi]=min(dist_l);


    if mv+tol<norm(x2-x1-d)
        d=dl(:,mi);
        e=x2-x1-d;
        a=max(-m*(e(3))/e(1),4);
        b=max(-m*(e(4))/e(2),4);
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
    xl1=[xl1 x1];
    ul1=[ul1 u1];
    xl2=[xl2 x2];
    ul2=[ul2 u2];
    el=[el e];
    u1l=[u1l, u1];
  
    lambdal=[lambdal lambda];
    %%%%%%%%switching rule
%  find di
%  
%     if  '?(conditioon  for switching)?' distance to di+ const < distance to d
%            K_lin=?????
%            d=???? 
%            end;0
end
%     U=[];
%     for i=1:3000
%         U=[U sqrt(ul(1,i)^2+ul(2,i)^2)];
%     end
    E=[];
    for i=1:3000
        E=[E sqrt(el(1,i)^2+el(2,i)^2)];
    end
figure(1);
hold on;

% 绘制第一行作为曲线
plot1=plot(tl, xl1(1,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$x_1$');
plot2=plot(tl, xl2(1,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$x_2$');
xlim([0 30])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$x$','FontSize', 20,'Interpreter','latex')
legend1=legend('$x_1$','$x_2$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;





figure(2);
hold on;

plot1=plot(tl, xl1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$y_1$');
plot2=plot(tl, xl2(2,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$y_2$');
xlim([0 30])
ylim([-0.8 1.2])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$y$','FontSize', 20,'Interpreter','latex')
legend1=legend('$y_1$','$y_2$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;

figure(3);
hold on;


% 绘制曲线
plot1=plot(xl1(1,:), xl1(2,:), 'r','LineWidth', 2);
set(plot1, 'DisplayName', '$r_1$');
plot2 = plot(xl2(1,:), xl2(2,:), 'b','LineWidth', 2);
set(plot2, 'DisplayName', '$r_2$');
ylim([-0.8 1.2])
xlabel('$x$','FontSize', 20,'Interpreter','latex')
ylabel('$y$','FontSize', 20,'Interpreter','latex')
legend('xl1','xl2');
legend1=legend('$r_1$','$r_2$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;



figure(4);
hold on;

plot3=plot(tl, ul2(1,:), 'r','LineWidth', 2);
ylim([-12 6])
set(plot3, 'DisplayName', '$u_x$');
plot3=plot(tl, ul2(2,:), 'b','LineWidth', 2);
ylim([-12 2])
set(plot3, 'DisplayName', '$u_x$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$u$','FontSize', 20,'Interpreter','latex')
legend1=legend('$u_x$','$u_y$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;

figure(5);
hold on;

plot4=plot(tl, el(1,:), 'r','LineWidth', 2);
set(plot4, 'DisplayName', '$e_x$');
plot5=plot(tl, el(2,:), 'b','LineWidth', 2);
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

plot3=plot(tl, E(1,:), 'r','LineWidth', 2);
ylim([0 3.5])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$e$','FontSize', 20,'Interpreter','latex')
set(plot3, 'DisplayName', '$e_{lin}$');
legend1=legend('$e_{hom}$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;

fprintf('\n===== Y-axis Tracking Summary (discrete polygon) =====\n');
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


