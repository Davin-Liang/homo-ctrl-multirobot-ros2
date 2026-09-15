function nx=hnorm_weight(x,L,rb,Q,J)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The function nx=hnorm_weight(x, L) computes (for a given vector x)                
%% a weighted homogeneous norm with the weights L=(r_1; r_2; ... r_n)>0
%% 
%% The function nx=hnorm_weight(x, L, rb) uses the smooting parameter rb: 
%% the larger rb, the smoother norm  (by default, rb=1)
%%
%%  
%% The function nx=hnorm_weight(x, L, rb, Q) uses the matrix  Q - (n x n)
%% to define the shape of the norm's level set (by default, Q is identity).
%%
%%  
%% The function nx=hnorm_weight(x, L, rb, Q, J) uses the matrix J - (n x n)
%% for the coordinate transformation x->Jx (by default, J is identity).
%%
%% Remark: To compute a homogeneous norm in the case of a diagonilizable 
%% linear dilation, the matrix J must be a real Jordan transformation of 
%% its generator Gd = J^{-1} * diag(L) * J  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n=size(x,1);
if nargin<3 rb=1; end;
if nargin<4 Q=eye(n); end;
if nargin<5 J=eye(n); end;


y=J*x;
y=abs(y).^(rb./L).*sign(y);
nx=(y'*Q*y)^(1/(2*rb));

