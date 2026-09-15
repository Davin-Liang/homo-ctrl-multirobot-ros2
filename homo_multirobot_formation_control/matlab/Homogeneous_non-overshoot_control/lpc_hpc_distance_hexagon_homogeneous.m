t=0; Tmax=30; 
h=0.01; % sampling period
m=2;%mass
A=[zeros(2) eye(2); zeros(2) zeros(2)];
B=[zeros(2); eye(2)]*1/m;
x1=[1;0;0;0];
x2=[5;1;0;0];

m_p=6; radius=1;
tol=0.1;
dl=[];
for i=0:m_p-1
 d=-radius*[cos(2*pi*i/m_p); sin(2*pi*i/m_p)];
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
lambda =diag([a b]);
k2=-2.4*lambda;
k1=lambda*(k2+lambda)/m;


k_lin=[k1/2 k2/2];
[K0,G0,P,nu_min,nu_max]=lpc2hpc(A,B,k_lin);
K=k_lin-K0;
nu=nu_min;
Gd=eye(4)+nu*G0;

tl=[];xl1=[];ul1=[];
xl2=[];ul2=[];
el=[];
hl=[];
ml=[];gl=[];

tic
while t<Tmax
  
    

    u1=-[eye(2) eye(2)]*x1+1*[cos(t);sin(t)];
    x1=x1+h*(A*x1+B*u1); 
    
    e=x2-x1-d;
    g=0.5*([eye(2) eye(2)]*x1-1*[cos(t);sin(t)]);
    nx=hnorm(e,Gd,P);
    %%     nx = max(alpha, min(beta,nx)) 
    u2=2*max(min(1,nx),0.05)^(1+nu)*K*expm(Gd*(1-log(max(min(1,nx),0.05))))*e; 
    x2=x2+h*(A*x2+B*u2);

    dist_l=[];
    for i=1:m_p  
        dist_l=[dist_l norm(x2-x1-dl(:,i))];
    end
    [mv mi]=min(dist_l);


    if mv+tol<norm(x2-x1-d)
        d=dl(:,mi);
        e=x2-x1-d;
        a=max(-m*(e(3))/e(1),1);
        b=max(-m*(e(4))/e(2),1);
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
    ml=[ml mi];
    hl=[hl nx];
    gl=[gl g];

    E=[];

end
toc

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
ylim([-1 1])
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
ylim([-1 2])
xlabel('$x$','FontSize', 14,'Interpreter','latex')
ylabel('$y$','FontSize', 14,'Interpreter','latex')
legend('xl1','xl2');
legend1=legend('leader robot','follower with homogeneous controller','FontSize', 14,'Box', 'off');
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

plot4=plot(tl, abs(el(1,:)), 'r','LineWidth', 2);
set(plot4, 'DisplayName', '$e_x$');
plot5=plot(tl, abs(el(2,:)), 'b','LineWidth', 2);
set(plot5, 'DisplayName', '$e_y$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$\ell$','FontSize', 20,'Interpreter','latex')
legend1=legend('$\ell_x$','$\ell_y$','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;




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
