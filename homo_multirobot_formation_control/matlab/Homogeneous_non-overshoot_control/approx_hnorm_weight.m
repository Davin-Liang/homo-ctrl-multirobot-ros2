function [Q L J]=approx_hnorm_weight(Gd,P,rb,Nmax)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function Q=approx_hnorm_weight(Gd,P,rb) computes the (n x n) matrix Q
%% for an explicit homogenenous norm 
%%
%%          ||x||_rb=(y'*Q*y)^(1/rb),     y=(J*x).^(rb./L)  
%%
%%  which approximates the canonical homogeneous norm induced by the norm 
%%          
%%                   ||x||=sqrt(x'*P*x) 
%% 
%% The (n x n) matrix Gd is a diagonalizable anti-Hurwitz generator of 
%% a linear dilation in R^n being strictly monotone with respect to ||x||
%%
%%                     Gd=inv(J)*diag(L)*J
%%
%% The symmetric positive definite (n x n) matrix P defines a shape of
%% the weighted Euclidean norm ||x|| 
%%
%% rb - a positive smoothing parameter
%%
%% Important: The function needs YALMIP toolbox with some LMI solver 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin<4 Nmax=10; end;
if nargin<3 rb=1; end;

n=size(P,1); 

[J L]=eig(Gd);
L=diag(L); J=inv(J);

%genaration of the array
z=[];
lin=[-1:n/Nmax:1];k=size(lin,2);
ind=ones(1,n); ok=1;
while ok==1
 x=[];if ind*ind'==n*k^2 ok=0; end;
 for i=1:n x=[x;lin(ind(i))]; end;
 if (min(ind)==1) || (max(ind)==k) z=[z J*x/sqrt(x'*P*x)]; end;
 if ind(1,1)<k ind(1,1)=ind(1,1)+1;
 else j=1;while (ind(1,j)==k) && (j<n) ind(1,j)=1; j=j+1; end;ind(1,j)=ind(1,j)+1;end;
end;


Lz=L*ones(1,size(z,2));
Phiz=abs(z).^(rb./L).*sign(z);
 
Q=sdpvar(n);
cost=@(Q)sum(sum((Q*Phiz).*Phiz).^2)-2*sum(sum((Q*Phiz).*Phiz));
ops = sdpsettings('verbose',0);
optimize([Q>=0],cost(Q),ops);

Q=double(Q);



