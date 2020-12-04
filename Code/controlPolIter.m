global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%%% INITIALIZATION POLICY
mu=zeros(K,1);
mu(:)=5;

%%% COMPUTATION OF G FOR THE INITIAL POLICY
GG=zeros(K,1);
for k=1:K
    GG(k)=G(k,mu(k));
end

%%% COMPUTATION OF P FOR INITIAL POLICY
PP=zeros(K);
for k=1:K
    for j=1:K
        PP(k,j)=P(k,j,mu(k));
    end
end

%%% POLICY ITERATION ALGORITHM

tol=0.5;
N=10000;
L=5;
I=eye(K);
JJ=zeros(K,N);
UU=zeros(K,1);
J=zeros(K,N,L);
Y=100;

invPP=eye(K);

for y=1:Y
    invPP=invPP+mpower(PP,y);
end

%JJ(:,1)=linsolve(I-PP,GG);
JJ(:,1)=invPP*GG;
Err=zeros(K,1);


for n=2:N % Iterations
    for k=1:K
        for u=1:L % Computation of cost to go for each input
            CTG=zeros(L,1);
            for j=1:K
                CTG(u)=CTG(u)+P(k,j,u)*JJ(j,n-1); % Expected cost to go
            end
            
            J(k,n,u)=G(k,u)+CTG(u); % Total cost for each input: E[step_cost] + E[cost to go]
            
        end
        
        [UU(k),mu(k)]=min(J(k,n,:));  % Optimization (minimization) over control inputs
        for j=1:K
            PP(k,j)=P(k,j,mu(k));
        end
        GG(k)=G(k,mu(k));
    end
    
    invPP=eye(K);
    
    for y=1:Y
        invPP=invPP+mpower(PP,y);
    end
    
   
    JJ(:,n)=invPP*GG;
    Check_var=0;
    Err=abs(JJ(:,n)-JJ(:,n-1));
    for k=1:K
        if Err(k)==0
            Check_var=1;
        end
    end
    if Check_var==0
        break
    end
end

J_opt=JJ(:,n);
u_opt_ind=mu;