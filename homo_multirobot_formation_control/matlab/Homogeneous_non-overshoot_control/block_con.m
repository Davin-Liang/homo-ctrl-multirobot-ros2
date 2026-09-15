function [T nt]=block_con(A,B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  The function [T n_list]=block_con(A,B) computes                  
%%  an invertible matrix T that transforms a controllable pair {A,B} 
%%  to the canonical block controlability form:                      
%%        T*A*inv(T)=[ 0  A12  0  ...   0  ;                         
%%                     0   0  A23 ...   0  ;                         
%%                    ... ... ... ...  ... ;                         
%%                     0   0  0 ...   Ak-1k;                         
%%                    Ak1 Ak2 Ak3 ...  Akk ]                         
%%  and                                                              
%%               T*B=[0; B0],                                        
%%  where                                                            
%%     - Aij is a block of the size (ni x nj) with i,j=1,...,k;      
%%     - B0 is a full row rank  matrix (nk x m).                     
%%  The array n_list=[n1,...,nk] contains sizes of the blocks.       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 T=0; nt=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----Ortogonal Transfomation------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[T nt]=trans_con(A,B);
if T==0 return; end;
k=size(nt,1);
ni=[1];j=1;
for i=1:k-1
    j=j+nt(i,1);
    ni=[ni;j];
end;
A=T*A*T';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----Triangular transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n=size(A,1);
Phi=eye(n);
for i=1:k-1
 temp_A=A(ni(i):ni(i)+nt(i)-1,ni(i)+nt(i):ni(i)+nt(i)+nt(i+1)-1);
 temp_block=[temp_A'*inv(temp_A*temp_A')*A(ni(i):ni(i)+nt(i)-1,1:ni(i)+nt(i)-1) eye(nt(i+1)) zeros(nt(i+1),n-ni(i)-nt(i)-nt(i+1)+1)];
 temp_T=eye(n);
 temp_T(ni(i+1):ni(i+1)+nt(i+1)-1,:)=temp_block;
 Phi=temp_T*Phi;
 A=(temp_T)*A*inv(temp_T);
end;
T=Phi*T;


