function [alpha beta gamma]=approx_hnorm_sigmoid(Gd,P,hn_fun,N,Nm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function Q=approx_hnorm(Gd,P,rb) computes paramters of universal 
%% approximator with sigmoid functions for the canonical homogeneous norm 
%% induced by the norm 
%%          
%%                   ||x||=sqrt(x'*P*x) 
%%
%%
%% The (n x n) matrix Gd is an anti-Hurwitz generator of a linear dilation
%%
%% The symmetric positive definite (n x n) matrix P defines a shape of
%% the weighted Euclidean norm ||x|| 
%%
%% hn_fun - a function which comupes an explict homogenenous norm
%%
%% N - number of sigmoid functions (by default, N=10)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin<5 Nm=10; end;

n=size(P,1); 

[J L]=eig(Gd);
L=diag(L); J=inv(J);

%genaration of the array
z=[];
lin=[-1:n/Nm:1];k=size(lin,2);
ind=ones(1,n); ok=1;
while ok==1
 x=[];if ind*ind'==n*k^2 ok=0; end;
 for i=1:n x=[x;lin(ind(i))]; end;
 if (min(ind)==1) || (max(ind)==k) z=[z x/sqrt(x'*P*x)]; end;
 if ind(1,1)<k ind(1,1)=ind(1,1)+1;
 else j=1;while (ind(1,j)==k) && (j<n) ind(1,j)=1; j=j+1; end;ind(1,j)=ind(1,j)+1;end;
end;

alpha=zeros(1,N);
beta=zeros(n,N);
for i=1:N
    alpha(i)=rand-0.5;
    beta(:,i)=rand(n,1)-0.5;
end;

M=size(z,2);
W=zeros(M,N);
for j=1:M
    nz=hn_fun(z(:,j)); pi_z=hproj(z(:,j),Gd,hn_fun);
    for i=1:N
      W(j,i)=nz/(1+exp(alpha(i)+beta(:,i)'*pi_z));
    end;
end;
gamma=pinv(W)*ones(M,1);
gamma=gamma';

