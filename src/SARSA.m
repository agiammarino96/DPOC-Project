function [Q,ra] = SARSA(map,StateSpace,P,initQ,epsilon,gamma,alpha,T,steps)
global TERMINAL_STATE_INDEX K
Q = initQ;
ra = zeros(T, 1);

for t=1:T
    disp(num2str(t));
    x=randi([1,K]);
    u = execute_policy(Q, x, epsilon);
    
    for tt=1:steps
        
        %get next state
        x_k_possible=find(P(x,:,u)~=0);
        while (isempty(x_k_possible))
            u=randi(5);
            x_k_possible=find(P(x,:,u)~=0);
        end
        prob=P(x,x_k_possible,u)/min(P(x,x_k_possible,u));
        for i=2:length(prob)
            prob(i)=prob(i)+prob(i-1);
        end
        draw=rand/min(P(x,x_k_possible,u));
        index_x_plus1=min(find(draw<prob));
        xP=x_k_possible(index_x_plus1);
        
        % learn
        reward = observe_reward(xP,StateSpace, map);
        uP = execute_policy(Q, xP, epsilon);
        ra(t)=gamma*ra(t)+reward;
        Q = update_Q(Q, x, u, ra(t), xP, uP, alpha,gamma);
        x=xP;
        u=uP;
        
        if x == TERMINAL_STATE_INDEX
            break
        end
    end
end

end

function a = execute_policy(Q, s, epsilon) % epsilon-greedy policy

temp = rand(1);
[~,I] = max(Q(s,:));

if epsilon>temp
    a = randi(5);
else
    a = I;
end

end

function r = observe_reward(sP,StateSpace, map)

global TERMINAL_STATE_INDEX
pickup=ComputePickUpStateIndex(StateSpace, map);
base = ComputeBaseStateIndex(StateSpace, map);
dropoff= TERMINAL_STATE_INDEX;

switch sP
    case pickup
        r=100;
    case dropoff
        r=100;
    case base
        r=-30;
    otherwise
        r=0;
end

end

function Q = update_Q(Q, s, a, r, sP, aP, alpha,gamma)

Q(s,a)=Q(s,a)+alpha*(r+gamma*Q(sP,aP)-Q(s,a));

end