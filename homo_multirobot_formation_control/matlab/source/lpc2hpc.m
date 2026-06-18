function [K0, G0, P, mu_min, mu_max]=lpc2hpc(A,B,K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function [K0, Gd, P, mu_min, mu_max]=lpc2hpc(A,B,K) upgrades 
%% the linear proportional controller (LPC) 
%%
%%            u=K*x         (x -system state)
%%
%% to Homogeneous Proportional Controller (HPC):     
%%                                                                      
%%   u=K0*x+nx^(1+mu)*(K-K0)*d(-log(nx))*x     (nx - homogeneous norm of x)          
%%                                                                       
%%
%% The input parameters:                                             
%%      A  - system matrix (n x n)                                       
%%      B  - control matrix (n x m)       
%%      K  - gain matrix (m x n) of LPC s.t. A+B*K is Hurwitz  
%%
%% The output parameters:                                             
%%      K0 - feedback gain (m x n) of linear component of HPC                                          
%%      G0 - matrix (n x n) that  defines the generator Gd=eye(n)+mu*G0  
%%           of the dilation d(s)=expm(s*Gd)                   
%%      P  - positive definite matrix which defines the canonical        
%%           homogeneous norm nx induced by the norm sqrt(x'*P*x) 
%%  mu_min - minimal admissible degree of HPC  (mu_min<=mu)  
%%  mu_max - maximal admissible degree of HPC  (mu<=mu_max)
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K0=0; G0=0; P=0; mu_min=0; mu_max=0;
tol=0.00001; 

% check dimensions
sA=size(A); sB=size(B); sK=size(K);

if sA(1)~=sA(2) disp('Error: the system matrix must be square'); return;
end;
if sB(1)~=sA(2) disp('Error:  dimensions of system  and control matrices must agree');return;
end;
if sK(1)~=sB(2) disp('Error:  dimensions of control and gain matrices must agree');return;
end;
n=sB(1); m=sB(2);

%controllability check
U=[B];tA=A;
k=0;
if rank(U)==n k=1;
end;i=1;
while (k==0) && (i<=n-1)
    i=i+1;
    U=[U tA*B];
    if rank(U)==n k=i; end;
    tA=A*tA;
end;
if k==0 disp('Error: The system is not controllable'); return;
end;
    
if det(U*U')<tol 
   disp('Warning: The system is weakly controllable. Parameters may be badly tuned due to computations issues.');
end;    

%% check is the closed-loop system is close to an unstable
rho=-max(real(eig(A+B*K)))*0.001;
if rho<tol disp('Error: The linear control system does not have a sufficient stability margin. The upgrade is impossible.'); return; end;

if rank(B)==n 
   K0=-B'*inv(B*B')*A;
   K=-B'*inv(B*B'); Gd=eye(n);
   P=eye(n);Gd=P;
   return;
end;   
    
%%%%%%%%%%%%%%%%%%%%%
% Design of control parameters K0, G0 using block decomposition
%%%%%%%%%%%%%%%%%%
[T nt]=block_con(A,B);
if T==0 K0=0; K=0;P=0; Gd=0; 
    disp('Error: A block decomposition is impossible.'); 
      return; end;
  
Anew=T*A*inv(T);
k=size(nt,1); 
%creation of an array of indecies
s=1;
n_ind=[s];
for i=1:k-1
s=s+nt(i);
n_ind=[n_ind; s];
end;

Bnew=T*B;
B0=Bnew(n_ind(k):n,1:m);
K0=-B0'*inv(B0*B0')*Anew(n_ind(k):n,1:n);
A0=Anew+Bnew*K0;
K0=K0*T;

vG0=[];
for i=1:k
 vG0=[vG0 (k-i)*ones(1,nt(i))];   
end;
G0=-inv(T)*diag(vG0)*T;

%% computation of P from Lyapunov equation (A+B*K)'*P+P(A+B*K)=-2*eye(n)
I_n=eye(n);
W0=[kron(I_n, (A+B*K)')+kron((A+B*K)',I_n)];
zet0=-[2*I_n(:)];
v_P=linsolve(W0,zet0);
P=reshape(v_P(1:n^2),n,n); 

%% calulations of admissible mu
lambda_min=min(real(eig(sqrtm(P)*G0*inv(sqrtm(P))+inv(sqrtm(P))*G0'*sqrtm(P))));
lambda_max=max(real(eig(sqrtm(P)*G0*inv(sqrtm(P))+inv(sqrtm(P))*G0'*sqrtm(P))));
if lambda_max>tol mu_min=max(-1,-1/lambda_max+tol);
else mu_min=-1;
end;
if lambda_min<-tol mu_max=min(1/k, -1/lambda_min);
else mu_max=1/k;
end;

