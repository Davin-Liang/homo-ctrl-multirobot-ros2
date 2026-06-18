function [Ah,Bh]=ZOH(h,A,B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function [Ah,Bh]=ZOH(h,A,B) computes Zero-Order-Hold discretization
%% 
%%   x(k+1)=Ah*x+Bh*u
%%
%%   of continuous-time linear system
%% 
%%   dx/dt=A*x+B*u
%%
%%  The input parameters:
%%  
%%     h  - sampling period
%%     A  - system matrix
%%     B  - control matrix
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tol=1e-20;


Ai=h*eye(size(A,1)); 
S=0;i=1;
while (norm(Ai)>tol)
S=S+Ai;i=i+1;
Ai=Ai*A*h/i;
end;
Bh=S*B;
Ah=eye(size(A,1))+A*S;



