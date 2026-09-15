function [z, s]=hproj(x,Gd,hn_fun)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function [z, s]=hproj(x,Gd,hn_fun) computes homogeneous projection                   
%% of a given non-zero vector x on the homogeneous unit sphere, namely,
%% it finds a real vector z and real scalar s such that 
%%                                                            
%% z=d(s)x and the homogeneous norm of z equals 1  (x is non-zero)  
%%                                                            
%% The input parameters are                                   
%%     x  - vector (p x 1) which homogeneous projection has to be computed  
%%    Gd  - anti-Hurwitx matrix (p x p)  being the generator of
%%          the dilation d(s)=expm(s*Gd) for any real s 
%% hn_fun - a function which computes a homogenenous norm 
%% 
%%
%%
%% The function [z, s]=hproj(x, Gd, P, hn_fun, tol) uses the additional 
%% parameter tol to control compuational tolerance (by default, tol=1e-6)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nx=hn_fun(x); s=0;z=0;
if nx==0 disp('Error: vector x must be nonzero'); 
else s=-log(nx); z=expm(s*Gd)*x; end;