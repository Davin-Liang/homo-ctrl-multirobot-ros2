function [T nt]=trans_con(A,B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  The function [T n_list]=trans_con(A,B) computes           %
%%  an orthogonal matrix T for a controllable pair {A,B}:     %
%%     T*A*T' = [ A11  A12   0  ...   0  ;                    %
%%                A21  A22  A23 ...   0  ;                    %
%%                ...  ...  ... ...  ... ;                    %
%%                 *    *    *   *  Ak-1k;                    %
%%                Ak1  Ak2  Ak3 ...  Akk ]                    %
%%  and                                                       %
%%              T*B=[0; B0],                                  %
%%  where                                                     %
%%    - Aij is a block of the size (ni x nj) with i,j=1,...,k %  
%%    - B0 is a full row rank  matrix (nk x m)                %
%%  The array n_list=[n1,...,nk] contains sizes of the blocks.%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n=size(A,1);nt=[]; T=0;
if n~=size(A,2) disp('Error: The matrix A is not square.'); return; end;
if n~=size(B,1) disp('Error: The number of rows of the matrix A does not coincide with the number of rows of the matrix B.'); return; end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-----Controlability check----------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U=[];Ak=eye(n);
for i=1:n
U=[U Ak*B];
Ak=Ak*A;
end;
if rank(U)<n disp('The pair {A,B} is not controlable.'); return;
end;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----Transformation------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=eye(n);
Ak=A;Bk=B;l=0;
while rank(Bk)<size(Ak,1)
    nt=[rank(Bk); nt];
    B_ort=null(Bk')';
    B_p=null(B_ort)';
    if size(Ak,1)<n T=[[B_ort;B_p] zeros(size(Ak,1),l); zeros(l,size(Ak,1)) eye(l)]*T;
    else T=[B_ort;B_p];
    end;
    l=l+rank(Bk);
    Bk=B_ort*Ak*B_p';
    Ak=B_ort*Ak*B_ort';        
end;
nt=[rank(Bk); nt];