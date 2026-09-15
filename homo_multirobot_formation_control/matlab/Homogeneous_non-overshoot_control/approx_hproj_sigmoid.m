function [xi Theta W]=approx_hproj_sigmoid(Gd,P,hn_fun,N,Nm)
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
%% 
%% Nm - defines number of points on the unit spheres as M=n^Nm 
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
    xi(i)=rand-0.5;
    Theta(:,i)=rand(n,1)-0.5;
end;

M=size(z,2);
G_=zeros(M,N);
p_z=[];
for j=1:M
    pi_z=expm(-log(hn_fun(z(:,j)))*Gd)*z(:,j);
    for i=1:N
      G_(j,i)=1/(1+exp(xi(i)+Theta(:,i)'*pi_z));
    end;
end;
iG=pinv(G_);
W=(iG*z')';
Theta=Theta';
xi=xi';

