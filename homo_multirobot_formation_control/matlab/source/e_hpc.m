function u=e_hpc(x, K0, K, Gd, P, mu, alpha, beta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function  u=e_hpc(x, K0, K, Gd, P, mu) descretizes explicitely
%% Homogeneous Proportional Controller (HPC)     
%%                                                                      
%%     u = K0*x + nx^(1+mu)*K*d(-ln(nx))*x 
%%
%%  where x - system state  and   nx - homogenenous norm of x
%%
%%
%% The function  u=e_hpc(x, K0, K, Gd, P, mu, alpha, beta)                    
%% uses the saturation parameters alpha and beta to lower/upper bound
%% the homogenenous norm nx     
%%                                                                      
%%     nx = max(alpha, min(beta,nx)) 
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if K0==0 u=zeros(size(K,1),1);
    else u=K0*x;
end;

if nargin==8 hn=hnorm(x,Gd,P,alpha,beta);
   elseif nargin==7 hn=hnorm(x,Gd,P,alpha);
   else hn=hnorm(x,Gd,P); 
end;

if hn>1e-20 %numerical tolerance
u=u+hn^(1+mu)*K*expm(-log(hn)*Gd)*x;
end;


