function [Q1,Q2,ra,n_steps,validation_reward,tot_n_valid] = Double_Q_Learning(P,initQ1,initQ2,epsilon,gamma,alpha,T,steps)
global TERMINAL_STATE_INDEX K BASE_STATE_INDEX
Q1 = initQ1;
Q2 = initQ2;
ra = zeros(T, 1);
n_steps = [];
validation_reward = [];
tot_n_valid = 0;

for t=1:T
    disp(num2str(t));
    x=randi([1,K]);
    
    
    for tt=1:steps
        
        choose_Q = rand;
        u = execute_policy((Q1+Q2)/2, x, epsilon);

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
        if (choose_Q < 0.5)
            uP = pick_greedy_action(Q1, xP);
        else
            uP = pick_greedy_action(Q2, xP);
        end
        % ra(t)=gamma*ra(t)+reward;
        ra(t) = reward;
        if (choose_Q < 0.5)
            Q1 = update_Q(Q2, x, u, ra(t), xP, uP, alpha,gamma);
        else
            Q2 = update_Q(Q1, x, u, ra(t), xP, uP, alpha,gamma);
        end
        x=xP;
        
        if x == TERMINAL_STATE_INDEX
            break
        end
    end
    
    if (mod(t,100) == 0)
        Q = (Q1 + Q2) / 2;
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

function a = execute_policy(Q, s, epsilon) % epsilon-greedy policy

temp = rand(1);
[~,I] = max(Q(s,:));

if epsilon>temp
    a = randi(5);
else
    a = I;
end

end

function a = pick_greedy_action(Q, s)

[~,a] = max(Q(s,:));

end

function r = observe_reward(s,sP)

global TERMINAL_STATE_INDEX PICKUP_STATE_INDEX BASE_STATE_INDEX

switch sP
    case PICKUP_STATE_INDEX
        r=100; % this is not used now
        % disp('PICKUP');
    case TERMINAL_STATE_INDEX
        r=100;
        % disp('DROPOFF');
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