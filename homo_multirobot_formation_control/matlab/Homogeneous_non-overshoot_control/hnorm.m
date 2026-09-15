function q=hnorm(x,Gd,P,alpha,beta,Nmax) %#codegen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function nx=hnorm(x, Gd, P) computes   (for a given vector x)                
%% a canonical d-homogeneous norm q induced by the weighted Euclidean norm           
%% nxp=sqrt(x'*P*x), namely:
%%                                                            
%% nx>0   such that  x'*d'(-log(nx))P*d(-log(nx))*x)=1    (if x is non-zero)
%%                                                            
%% The input parameters are                                   
%%     x - vector (p x 1) which norm has to be computed  
%%    Gd - anti-Hurwitx matrix (p x p)  being the generator of
%%          the dilation d(s)=expm(s*Gd) for any real s 
%%    P  - positive definite matrix (p x p) such that P*Gd+Gd'*P>0 
%% 
%%
%%
%% Remark: 
%% The function hnorm is not optimized for a real-time compuation of 
%% the homogeneous norm in practice. To design an fast algorithm for  
%% practical application of a homogeneous control please do contanct  
%%            andrey.polyakov@inria.fr (HPC_toolbox developer)
%%
%%
%%
%% The function q=h_norm(x, Gd, P, alpha, beta) uses
%% additional parameters alpha and beta to lower/upper bound
%% the homogeneous norm as follows: 
%%
%%     q=max(alpha,min(q,beta)); 
%%
%% The function q=h_norm(x, Gd, P, alpha, beta, Nmax) uses
%% additional parameter Nmax to define the maximum number of iterations of 
%% the numerical method (Nmax=20, by default)
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin<6 Nmax=20; end; 
if norm(x)>1e-16
tol=1e-6;
a=-1; q=0; 
y=expm(-Gd*a)*x;
while (y'*P*y<1) && (a>-746) 
a=a*2;
y=expm(-Gd*a)*x;
end;
if y'*P*y>1
    b=1;
    y=expm(-Gd*b)*x;
    while (y'*P*y>1) && (b<710) 
        b=b*2;
        y=expm(-Gd*b)*x;
    end;
    if y'*P*y<1
    c=(a+b)/2;
    y=expm(-Gd*c)*x;
    Qf=y'*P*y-1; i=0;
    while (abs(Qf)>tol) && (i<Nmax)
        i=i+1;
        if Qf>0 a=c;
        else b=c;
        end;
        c=(a+b)/2;
        y=expm(-Gd*c)*x;
        Qf=y'*P*y-1;
    end;
    else c=b;
    end;
else c=a;
end;
q=exp(c);
else q=0;
end;
if nargin>=5 q=min(beta,q); end;   
if nargin>=4 q=max(alpha,q);end;  
