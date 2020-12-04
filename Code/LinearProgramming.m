function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
Nu=5;                % Number of control inputs
M=[];     % Matrix that multiplies V for constraint equation
                % Counter for constraint equations
b=[];     % known vector of constraint equations
I_c=eye(K);
I_c(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX)=0;
for k=1:(TERMINAL_STATE_INDEX-1)
    D=zeros(1,K);
    D(1,k)=1;
    for u=1:Nu
        if G(k,u)~=Inf
            M(end+1,:)=D-P(k,:,u)*I_c;
            b(end+1)=G(k,u);
        end
    end
end


for k=(TERMINAL_STATE_INDEX+1):K
    D=zeros(1,K);
    D(1,k)=1;
    for u=1:Nu
        if G(k,u)~=Inf
            M(end+1,:)=D-P(k,:,u)*I_c;
            b(end+1)=G(k,u);
        end
    end
end
f=ones(K,1);
f(TERMINAL_STATE_INDEX)=0;
J_opt = linprog(-f,M,b);

%%% COMPUTATION OF OPTIMAL POLICY USING ONE ITERATION OF VALUE ITERATION
JJ=zeros(K,Nu);
JJJ=zeros(K,1);
mu=zeros(K,1);
for k=1:K % For each state, we have to perform the optimization
    
    
    CTG=zeros(Nu,1); % Cost To Go set to zero for each input
    
    for u=1:Nu % Computation of cost to go for each input
        t=1;
        for j=1:(TERMINAL_STATE_INDEX-1)
            CTG(u)=CTG(u)+P(k,j,u)*J_opt(t); % Expected cost to go
            t=t+1;
        end
        for j=(TERMINAL_STATE_INDEX+1):K
            CTG(u)=CTG(u)+P(k,j,u)*J_opt(t); % Expected cost to go
            t=t+1;
        end
        
        JJ(k,u)=G(k,u)+CTG(u); % Total cost for each input: E[step_cost] + E[cost to go]
        
    end
    
    [JJJ(k),mu(k)]=min(JJ(k,:));  % Optimization (minimization) over control inputs
end

u_opt_ind=mu;
u_opt_ind(TERMINAL_STATE_INDEX)=HOVER;


end




