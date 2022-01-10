function [Q,ra,n_steps,validation_reward,tot_n_valid] = SARSA_UCB(P,initQ,gamma,alpha,T,steps)
global TERMINAL_STATE_INDEX K BASE_STATE_INDEX L
Q = initQ;
ra = zeros(T, 1);
n_steps = [];
validation_reward = [];
tot_n_valid = 0;
N_count = ones(K,L);
inv_p = 1;
U_nom = sqrt(log(inv_p))*ones(K,L);
U_den = sqrt(2*N_count);
U = U_nom./U_den;

for t=1:T
    x = randi([1 K]);
    u = pick_greedy_action(Q+U, x);
    %inv_p = inv_p+1;
    
    for tt=1:steps
        
        inv_p = inv_p+1;
        N_count(x,u) = N_count(x,u) +1;
        U_nom = sqrt(log(inv_p))*ones(K,L);
        U_den(x,u) = sqrt(2*N_count(x,u));
        U = U_nom./U_den;
        
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
        reward = observe_reward(x,xP);
        uP = pick_greedy_action(Q+U, xP);
        % ra(t)=gamma*ra(t)+reward;
        ra(t) = reward;
        Q = update_Q(Q, x, u, ra(t), xP, uP, alpha,gamma);
        x=xP;
        u=uP;
        
        if x == TERMINAL_STATE_INDEX
            break
        end
    end
    
    if (mod(t,100) == 0)
        disp(['iter: ' , num2str(t)]);
        tot_n_valid = tot_n_valid + 10;
        for n_validations = 1:10
            
            s = BASE_STATE_INDEX;
            cum_reward = 0;
            
            for tt=1:steps
                
                a = pick_greedy_action(Q, s);
                %get next state
                s_k_possible=find(P(s,:,a)~=0);
                while (isempty(s_k_possible))
                    a=randi(5);
                    s_k_possible=find(P(s,:,a)~=0);
                end
                prob=P(s,s_k_possible,a)/min(P(s,s_k_possible,a));
                for i=2:length(prob)
                    prob(i)=prob(i)+prob(i-1);
                end
                draw=rand/min(P(s,s_k_possible,a));
                index_s_plus1=min(find(draw<prob));
                sP=s_k_possible(index_s_plus1);
                
                % learn
                cum_reward = cum_reward + observe_reward(s,sP);
                s=sP;
                if s == TERMINAL_STATE_INDEX || tt == steps
                    n_steps(end+1) = tt;
                    validation_reward(end+1) = cum_reward;
                    break
                end
            end
            
        end
        
    end
    
end

end

function a = pick_greedy_action(Q, s)

[~,a] = max(Q(s,:));

end

function r = observe_reward(s,sP)

global TERMINAL_STATE_INDEX PICKUP_STATE_INDEX BASE_STATE_INDEX

switch sP
    case PICKUP_STATE_INDEX
        r=100;
    case TERMINAL_STATE_INDEX
        r=100;
    case BASE_STATE_INDEX
        if s == BASE_STATE_INDEX
            r=0;
        else
            r=-30;
        end
    otherwise
        r=0;
end

end

function Q = update_Q(Q, s, a, r, sP, aP, alpha,gamma)

Q(s,a)=Q(s,a)+alpha*(r+gamma*Q(sP,aP)-Q(s,a));

end