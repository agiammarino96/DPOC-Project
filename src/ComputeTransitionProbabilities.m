function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND L
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX


P=zeros(K,K,L);                         % Set to zero the transition probability matrix
[M,N]=size(map);                        % size of the world
[i_base,j_base]=find(map==BASE);        % (i,j) for the base cell
[i_pickup,j_pickup]=find(map==PICK_UP); % (i,j) for pickup cell


for psi=0:1
    for i=1:M
        for j=1:N
            
            k=find(ismember(stateSpace, [i j psi],'rows')); % Initial state
            
            
            if map(i,j)~=TREE
                %%% Initial state correspondent with terminal state and...
                %%% cells correspondent to trees not considered
                
                if k==TERMINAL_STATE_INDEX
                else
                    
                    for u=1:L
                        
                        %%% Only one of the next cases should be satisfied
                        comp_no=1; % 
                        %%%%% EAST CASE
                        if j~=N
                            if u==EAST && map(i,j+1)~=TREE %%% North can be chosen
                                r=i;    % auxiliary coordinate for i
                                s=j+1;  % auxiliary coordinate for j
                                comp_no=0;
                            end
                        elseif j==N && u==EAST
                            comp_no=1;
                        end
                        
                        %%%%% WEST CASE
                        if j~=1
                            if u==WEST && map(i,j-1)~=TREE %%% South can be chosen
                                r=i;
                                s=j-1;
                                comp_no=0;
                            end
                        elseif j==1 && u==WEST
                            comp_no=1;
                        end
                        
                        %%%%% SOUTH CASE
                        if i~=1
                            if u==SOUTH && map(i-1,j)~=TREE %%% West can be chosen
                                r=i-1;
                                s=j;
                                comp_no=0;
                            end
                        elseif i==1 && u==SOUTH
                            comp_no=1;
                        end
                        
                        %%%%% NORTH CASE
                        if i~=M
                            if u==NORTH && map(i+1,j)~=TREE %%% East can be chosen
                                r=i+1;
                                s=j;
                                comp_no=0;
                            end
                        elseif i==M && u==NORTH
                            comp_no=1;
                        end
                        
                        %%%%% HOVER CASE
                        if u==HOVER  %%% Hover can always be chosen
                            r=i;
                            s=j;
                            comp_no=0;
                        end
                        
                        if comp_no==0
                            
                            
                            t=find(ismember(stateSpace, [r s psi],'rows')); % State after input "u"
                            
                            %%% Definition of coordinates around the cell (achieved
                            %%% after the control input) from where the shooters
                            %%% can shoot the drone
                            
                            z=s;
                            y=r;
                            yy=r+1;
                            zz=s+1;
                            zzz=s+2;
                            yyy=r+2;
                            nyy=r-1;
                            nyyy=r-2;
                            nzz=s-1;
                            nzzz=s-2;
                            %%%%%% CASE OF NO WIND
                            
                            %%% build the probability of being shooted or not
                            
                            
                            p_nh=1;
                            
                            if y<=M && z<=N && y>=1 && z>=1
                                if map(y,z)==SHOOTER % If there is the shooter in the cell I'm checking
                                    p=GAMMA;           % Prob of shooted
                                    p_nh=p_nh*(1-p);   % Building Prob of not shooted
                                end
                            end
                            
                            if y<=M && zz<=N && y>=1 && zz>=1
                                if map(y,zz)==SHOOTER
                                    p=GAMMA/(1+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if y<=M && zzz<=N && y>=1 && zzz>=1
                                if map(y,zzz)==SHOOTER
                                    p=GAMMA/(2+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if yy<=M && z<=N && yy>=1 && z>=1
                                if map(yy,z)==SHOOTER
                                    p=GAMMA/(1+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if yyy<=M && z<=N && yyy>=1 && z>=1
                                if map(yyy,z)==SHOOTER
                                    p=GAMMA/(1+2);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if yy<=M && zz<=N && yy>=1 && zz>=1
                                if map(yy,zz)==SHOOTER
                                    p=GAMMA/(1+2);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if y<=M && nzz<=N && y>=1 && nzz>=1
                                if map(y,nzz)==SHOOTER
                                    p=GAMMA/(1+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if y<=M && nzzz<=N && y>=1 && nzzz>=1
                                if map(y,nzzz)==SHOOTER
                                    p=GAMMA/(1+2);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if nyy<=M && z<=N && nyy>=1 && z>=1
                                if map(nyy,z)==SHOOTER
                                    p=GAMMA/(1+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if nyyy<=M && z<=N && nyyy>=1 && z>=1
                                if map(nyyy,z)==SHOOTER
                                    p=GAMMA/(2+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if nyy<=M && zz<=N && nyy>=1 && zz>=1
                                if map(nyy,zz)==SHOOTER
                                    p=GAMMA/(2+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if nyy<=M && nzz<=N && nyy>=1 && nzz>=1
                                if map(nyy,nzz)==SHOOTER
                                    p=GAMMA/(2+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            if yy<=M && nzz<=N && yy>=1 && nzz>=1
                                if map(yy,nzz)==SHOOTER
                                    p=GAMMA/(2+1);
                                    p_nh=p_nh*(1-p);
                                end
                            end
                            
                            
                            
                            pickup0=find(ismember(stateSpace, [i_pickup j_pickup 0],'rows'));
                            pickup1=find(ismember(stateSpace, [i_pickup j_pickup 1],'rows'));
                            if t==pickup0
                                P(k,pickup1,u)=P(k,pickup1,u)+(1-P_WIND)*p_nh; % If you end up in pickup0, psi becomes 1
                                P(k,pickup0,u)=0;
                            else
                                P(k,t,u)=P(k,t,u)+(1-P_WIND)*p_nh; % No shooted and no wind
                            end
%                             if k==pickup0
%                                 P(k,t,u)=0;
%                             end
                            
                            base0=find(ismember(stateSpace, [i_base j_base 0],'rows'));
                            P(k,base0,u)=P(k,base0,u)+(1-P_WIND)*(1-p_nh); % Shooted and no wind
                            
                            %%%%%% Case wind
                            
                            
                            % North wind
                            if s+1>N || map(r,s+1)==TREE
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25; % wind causes crash
                            else % if wind doesn't cause crash
                                t=find(ismember(stateSpace, [r s+1 psi],'rows'));
                                s_nord=s+1;
                                
                                z=s_nord;
                                y=r;
                                yy=r+1;
                                zz=s_nord+1;
                                zzz=s_nord+2;
                                yyy=r+2;
                                nzzz=s_nord-2;
                                nyy=r-1;
                                nyyy=r-2;
                                nzz=s_nord-1;
                                
                                
                                %%% build the probability of being shooted or not
                                
                                
                                
                                p_nh=1;
                                if y<=M && z<=N && y>=1 && z>=1
                                    if map(y,z)==SHOOTER % If there is the shooter in the cell I'm checking
                                        p=GAMMA;    % Prob of shooted
                                        p_nh=p_nh*(1-p);   % Prob of not shooted
                                    end
                                end
                                if y<=M && zz<=N && y>=1 && zz>=1
                                    if map(y,zz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && zzz<=N && y>=1 && zzz>=1
                                    if map(y,zzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && z<=N && yy>=1 && z>=1
                                    if map(yy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yyy<=M && z<=N && yyy>=1 && z>=1
                                    if map(yyy,z)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && zz<=N && yy>=1 && zz>=1
                                    if map(yy,zz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzz<=N && y>=1 && nzz>=1
                                    if map(y,nzz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzzz<=N && y>=1 && nzzz>=1
                                    if map(y,nzzz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && z<=N && nyy>=1 && z>=1
                                    if map(nyy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyyy<=M && z<=N && nyyy>=1 && z>=1
                                    if map(nyyy,z)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && zz<=N && nyy>=1 && zz>=1
                                    if map(nyy,zz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && nzz<=N && nyy>=1 && nzz>=1
                                    if map(nyy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && nzz<=N && yy>=1 && nzz>=1
                                    if map(yy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                
                                if t==pickup0                                                               
                                    P(k,pickup1,u)=P(k,pickup1,u)+P_WIND*0.25*p_nh; % If you end up in pickup0, psi becomes 1
                                    P(k,t,u)=0;
                                else
                                  P(k,t,u)=P(k,t,u)+P_WIND*0.25*p_nh; % North wind, no hit   
                                end
                                
%                                 if k==pickup0
%                                     P(k,t,u)=0;
%                                 end
                                
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25*(1-p_nh); % North wind, hit
                            end
                            
                            
                            
                            % South wind
                            if s-1<1 || map(r,s-1)==TREE
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25;
                            else
                                t=find(ismember(stateSpace, [r s-1 psi],'rows'));
                                
                                s_south=s-1;
                                
                                z=s_south;
                                y=r;
                                yy=r+1;
                                zz=s_south+1;
                                zzz=s_south+2;
                                yyy=r+2;
                                nzzz=s_south-2;
                                nyy=r-1;
                                nyyy=r-2;
                                nzz=s_south-1;
                                
                                
                                %%% build the probability of being shooted or not
                                
                                
                                p_nh=1;
                                if y<=M && z<=N && y>=1 && z>=1
                                    if map(y,z)==SHOOTER % If there is the shooter in the cell I'm checking
                                        p=GAMMA;    % Prob of shooted
                                        p_nh=p_nh*(1-p);   % Prob of not shooted
                                    end
                                end
                                if y<=M && zz<=N && y>=1 && zz>=1
                                    if map(y,zz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && zzz<=N && y>=1 && zzz>=1
                                    if map(y,zzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && z<=N && yy>=1 && z>=1
                                    if map(yy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yyy<=M && z<=N && yyy>=1 && z>=1
                                    if map(yyy,z)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && zz<=N && yy>=1 && zz>=1
                                    if map(yy,zz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzz<=N && y>=1 && nzz>=1
                                    if map(y,nzz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzzz<=N && y>=1 && nzzz>=1
                                    if map(y,nzzz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && z<=N && nyy>=1 && z>=1
                                    if map(nyy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyyy<=M && z<=N && nyyy>=1 && z>=1
                                    if map(nyyy,z)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && zz<=N && nyy>=1 && zz>=1
                                    if map(nyy,zz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && nzz<=N && nyy>=1 && nzz>=1
                                    if map(nyy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && nzz<=N && yy>=1 && nzz>=1
                                    if map(yy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                
                                if t==pickup0                                                               
                                    P(k,pickup1,u)=P(k,pickup1,u)+P_WIND*0.25*p_nh; % If you end up in pickup0, psi becomes 1
                                    P(k,t,u)=0;
                                else
                                  P(k,t,u)=P(k,t,u)+P_WIND*0.25*p_nh; % North wind, no hit   
                                end
                                
%                                 if k==pickup0
%                                     P(k,t,u)=0;
%                                 end
                                
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25*(1-p_nh); % South wind, hit
                            end
                            
                            
                            % East wind
                            if r+1>M || map(r+1,s)==TREE
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25;
                            else
                                
                                t=find(ismember(stateSpace, [r+1 s psi],'rows'));
                                r_east=r+1;
                                z=s;
                                y=r_east;
                                yy=r_east+1;
                                zz=s+1;
                                zzz=s+2;
                                yyy=r_east+2;
                                nzzz=s-2;
                                nyy=r_east-1;
                                nyyy=r_east-2;
                                nzz=s-1;
                                
                                
                                %%% build the probability of being shooted or not
                                
                                
                                p_nh=1;
                                if y<=M && z<=N && y>=1 && z>=1
                                    if map(y,z)==SHOOTER % If there is the shooter in the cell I'm checking
                                        p=GAMMA;    % Prob of shooted
                                        p_nh=p_nh*(1-p);   % Prob of not shooted
                                    end
                                end
                                if y<=M && zz<=N && y>=1 && zz>=1
                                    if map(y,zz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && zzz<=N && y>=1 && zzz>=1
                                    if map(y,zzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && z<=N && yy>=1 && z>=1
                                    if map(yy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yyy<=M && z<=N && yyy>=1 && z>=1
                                    if map(yyy,z)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && zz<=N && yy>=1 && zz>=1
                                    if map(yy,zz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzz<=N && y>=1 && nzz>=1
                                    if map(y,nzz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzzz<=N && y>=1 && nzzz>=1
                                    if map(y,nzzz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && z<=N && nyy>=1 && z>=1
                                    if map(nyy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyyy<=M && z<=N && nyyy>=1 && z>=1
                                    if map(nyyy,z)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && zz<=N && nyy>=1 && zz>=1
                                    if map(nyy,zz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && nzz<=N && nyy>=1 && nzz>=1
                                    if map(nyy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && nzz<=N && yy>=1 && nzz>=1
                                    if map(yy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if t==pickup0                                                               
                                    P(k,pickup1,u)=P(k,pickup1,u)+P_WIND*0.25*p_nh; % If you end up in pickup0, psi becomes 1
                                    P(k,t,u)=0;
                                else
                                  P(k,t,u)=P(k,t,u)+P_WIND*0.25*p_nh; % North wind, no hit   
                                end
                                
%                                 if k==pickup0
%                                     P(k,t,u)=0;
%                                 end
                                
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25*(1-p_nh); % East wind, hit
                            end
                            
                            
                            
                            % West wind
                            if r-1<1 || map(r-1,s)==TREE
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25;
                            else
                                
                                t=find(ismember(stateSpace, [r-1 s psi],'rows'));
                                r_west=r-1;
                                z=s;
                                y=r_west;
                                yy=r_west+1;
                                zz=s+1;
                                zzz=s+2;
                                yyy=r_west+2;
                                nzzz=s-2;
                                nyy=r_west-1;
                                nyyy=r_west-2;
                                nzz=s-1;
                                
                                
                                %%% build the probability of being shooted or not
                                p_nh=1;
                                if y<=M && z<=N && y>=1 && z>=1
                                    if map(y,z)==SHOOTER % If there is the shooter in the cell I'm checking
                                        p=GAMMA;    % Prob of shooted
                                        p_nh=p_nh*(1-p);   % Prob of not shooted
                                    end
                                end
                                if y<=M && zz<=N && y>=1 && zz>=1
                                    if map(y,zz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && zzz<=N && y>=1 && zzz>=1
                                    if map(y,zzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && z<=N && yy>=1 && z>=1
                                    if map(yy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yyy<=M && z<=N && yyy>=1 && z>=1
                                    if map(yyy,z)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && zz<=N && yy>=1 && zz>=1
                                    if map(yy,zz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzz<=N && y>=1 && nzz>=1
                                    if map(y,nzz)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if y<=M && nzzz<=N && y>=1 && nzzz>=1
                                    if map(y,nzzz)==SHOOTER
                                        p=GAMMA/(1+2);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && z<=N && nyy>=1 && z>=1
                                    if map(nyy,z)==SHOOTER
                                        p=GAMMA/(1+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyyy<=M && z<=N && nyyy>=1 && z>=1
                                    if map(nyyy,z)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && zz<=N && nyy>=1 && zz>=1
                                    if map(nyy,zz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if nyy<=M && nzz<=N && nyy>=1 && nzz>=1
                                    if map(nyy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                if yy<=M && nzz<=N && yy>=1 && nzz>=1
                                    if map(yy,nzz)==SHOOTER
                                        p=GAMMA/(2+1);
                                        p_nh=p_nh*(1-p);
                                    end
                                end
                                
                                if t==pickup0                                                               
                                    P(k,pickup1,u)=P(k,pickup1,u)+P_WIND*0.25*p_nh; % If you end up in pickup0, psi becomes 1
                                    P(k,t,u)=0;
                                else
                                  P(k,t,u)=P(k,t,u)+P_WIND*0.25*p_nh; % North wind, no hit   
                                end
                                
%                                 if k==pickup0
%                                     P(k,t,u)=0;
%                                 end
                                
                                P(k,base0,u)=P(k,base0,u)+P_WIND*0.25*(1-p_nh); % West wind, hit
                            end
                        end
                    end
                end
            end
        end
    end
end

k=TERMINAL_STATE_INDEX;

for t=1:K
    if t~=k
        for u=NORTH:HOVER
            P(k,t,u)=0;
        end
    else
        for u=NORTH:HOVER
            P(k,t,u)=1;
        end
    end
end


end

