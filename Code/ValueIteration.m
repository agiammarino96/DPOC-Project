function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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

L=5;        % Number of control inputs
tol=10^(-5);  % tolerance

       
V=zeros(K,L); % Cost for the specific control input 
VV=zeros(K,2);  % Cost optimized over the control inputs
I=zeros(K,1); % indexes
Err=zeros(K,1); % Errors between current and previous iteration for each state

%%% INITIALIZATION

VV(:,1)=50;
VV(TERMINAL_STATE_INDEX,1)=0;
          
n=0;     
Check_err=1;

while Check_err==1 % high number of iterations, if error<tolerance we exit from the loop
    
    n=n+1; % next time step
    Check_err=0; % Boolean for the error checking
    
    for k=1:K % For each state, we have to perform the optimization
        if n>1
            VV(:,1)=VV(:,2);
        end
        
        if k==TERMINAL_STATE_INDEX
            VV(k,2)=0; % Cost optimized is always zero for terminal state            
            V(k,:)=0;  % Cost given by every input is zero for terminal state              
                        
        else
            % Here we have to write the optimization for the single state
            
            CTG=zeros(L,1); % Cost To Go set to zero for each input
            
            for u=1:L % Computation of cost to go for each input
                
                for j=1:K
                    CTG(u)=CTG(u)+P(k,j,u)*VV(j,1); % Expected cost to go
                end
                
                V(k,u)=G(k,u)+CTG(u); % Total cost for each input: E[step_cost] + E[cost to go]
                
            end
            
            [VV(k,2),I(k)]=min(V(k,:));  % Optimization (minimization) over control inputs         
        end

       
          % Check error between previous and current iteration:
          Err(k)=abs(VV(k,2)-VV(k,1));
          
          % If at least one state has not coverged yet, the Boolean is set
          % to 1 and another iteration will be performed. Only if all the
          % errors are lower than the tolerance, the Boolean remains null
          % and the algorithm finishes.
          
          if Err(k)>tol
              Check_err=1;
          end
             
    end
    % After having optimized each state (k), if check error is zero it
    % means that non of the states is above the tolerance --> exit from
    % algorithm
    
end
J_opt=VV(:,2); % Values optimized at last iteration performed
u_opt_ind=I; % Indexes obtained at last iteration performed

end