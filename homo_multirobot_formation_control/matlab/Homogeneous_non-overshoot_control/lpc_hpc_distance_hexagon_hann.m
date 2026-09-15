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

rb=1;
[Q L J]=approx_hnorm_weight(Gd,P,rb);
hn_fun=@(x)hnorm_weight(x,L,rb,Q,J);
[alpha beta gamma]=approx_hnorm_sigmoid(Gd,P,hn_fun,10); 
hx_fun=@(x)hnorm_sigmoid(x,Gd,alpha,beta,gamma,hn_fun);

[Ah, Bh]=ZOH(h,A,B);

tll=[];xll1=[];ull1=[];
xll2=[];ull2=[];
ell=[];
hl=[];
ml=[];gl=[];hxl=[];Ell=[];

tic
while t<Tmax
  
    
    e=x2-x1-d;

    
    u1=-[eye(2) eye(2)]*x1+1*[cos(t);sin(t)];
    x1=x1+h*(A*x1+B*u1); 
    

    
    
    hx=hx_fun(e);
    
    %%     nx = max(alpha, min(beta,nx)) 
   
    u2=2*max(min(1,hx),0.05)^(1+nu)*K*expm(Gd*(1-log(max(min(1,hx),0.05))))*e; 



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
    E1=sqrt(e(1)^2+e(2)^2);
    tll=[tll t];
    xll1=[xll1 x1];
    ull1=[ull1 u1];
    xll2=[xll2 x2];
    ull2=[ull2 u2];
    ell=[ell e];
    ml=[ml mi];
    hxl=[hxl hx];
    Ell=[Ell E1];


    E=[];

end
toc


figure(1);
hold on;

% 绘制第一行作为曲线
plot1=plot(tll, xll1(1,:), 'y--','LineWidth', 2);
set(plot1, 'DisplayName', '$x_1$');
plot2=plot(tll, xll2(1,:), 'g--','LineWidth', 2);
set(plot2, 'DisplayName', '$x_2$');
xlim([0 30])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$x$','FontSize', 20,'Interpreter','latex')
legend1=legend('$x_1$-hann','$x_2$-hann','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;



figure(2);
hold on;

plot1=plot(tll, xll1(2,:), 'y--','LineWidth', 2);
set(plot1, 'DisplayName', '$y_1$');
plot2=plot(tll, xll2(2,:), 'g--','LineWidth', 2);
set(plot2, 'DisplayName', '$y_2$');
xlim([0 30])
ylim([-1 1])
xlabel('$t(s)$','FontSize', 20,'Interpreter','latex')
ylabel('$y$','FontSize', 20,'Interpreter','latex')
legend1=legend('$y_1$-hann','$y_2$-hann','FontSize', 20,'Box', 'off');
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
legend1=legend('leader robot','follower with homogeneous controller-HANN','FontSize', 14,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;


figure(4);
hold on;

plot3=plot(tll, ull2(1,:), 'y--','LineWidth', 2);
ylim([-12 6])
set(plot3, 'DisplayName', '$u_x$');
plot3=plot(tll, ull2(2,:), 'g--','LineWidth', 2);
ylim([-12 2])
set(plot3, 'DisplayName', '$u_y$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$u$','FontSize', 20,'Interpreter','latex')
legend1=legend('$u_x$-hann','$u_y$-hann','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;



figure(5);
hold on;

plot4=plot(tll, abs(ell(1,:)), 'y--','LineWidth', 2);
set(plot4, 'DisplayName', '$e_x$');
plot5=plot(tll, abs(ell(2,:)), 'g--','LineWidth', 2);
set(plot5, 'DisplayName', '$e_y$');
xlabel('$t$','FontSize', 20,'Interpreter','latex')
ylabel('$\ell$','FontSize', 20,'Interpreter','latex')
legend1=legend('$\ell_x$-hann','$\ell_y$-hann','FontSize', 20,'Box', 'off');
set(legend1,'Interpreter','latex');
grid on;
