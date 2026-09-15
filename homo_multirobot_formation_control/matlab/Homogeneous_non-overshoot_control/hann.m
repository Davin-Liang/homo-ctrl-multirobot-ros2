function y=hann(x, A, B, C, Gd, nu, hn_fun, a_fun)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function y=hann(x, A, B, C, Gd, nu, hn_fun)                    
%% computes the value of homogeneous ANN     
%%                                      
%%      y=||x||_d^nu*C*sigma(A*d(-ln(||x||_d))*x+B)
%%
%% where sigma(r)=1/(1+exp(r))
%%
%% The input parameters: 
%%     x  -  vector (n x 1)             - argument of the function 
%%     A  -  matrix (N x n)         \                              
%%     B  -  vector (N x 1)          >  - parameters of ANN
%%     C  -  matrix (1 x N)         /
%%     Gd -  matrix (n x n)             - generator of dilation
%%     nu -  scalar (1 x 1)             - homogeneity degree 
%% hn_fun -  @ function                 - homogeneous norm
%%                                                                      
%% The output parameters:                                                
%%     y - scalar (1 x 1)               - value of the function
%%
%%
%% The function y=hann(x, A, B, C, Gd, nu, hn_fun, a_fun)                    
%% computes the value of homogeneous ANN 
%%
%% y=||x||_d^nu*C*a_fun(A*d(-ln(||x||_d))*x+B)
%%
%% where a_fun is a user defined activation function (sigmoid by default) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hn=hn_fun(x); 
if nargin<8 a_fun=@(r)1./(1+exp(r)); end;
y=hn^nu*C*a_fun(A*expm(-Gd*log(hn))*x+B);