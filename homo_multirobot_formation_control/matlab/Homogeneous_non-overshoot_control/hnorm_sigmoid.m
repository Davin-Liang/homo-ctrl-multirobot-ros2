function nx=hnorm_sigmoid(x,Gd,alpha,beta,gamma,hn_fun)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function nx=hnorm_sigmoid(x, Gd, alpha, beta, gamma, hn_fun) 
%% computes (for a given vector x) a canonical homogeneous norm 
%% using universal approximator
%% 
%% The (n x n) matrix Gd is a generator of linear dilation
%%
%% alpha, beta, gamma are parameter of universal approximator
%%
%% hn_fun a function which computes an explicit homogeneous norm 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nz=hn_fun(x); pi_z=expm(-log(nz)*Gd)*x;
nx=0;
for i=1:size(alpha,2)
nx=nx+gamma(1,i)/(1+exp(alpha(1,i)+beta(:,i)'*pi_z));
end;
nx=nx*nz;


