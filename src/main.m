% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2019
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Nikita Rudin
% rudinn@ethz.ch
% David Hoeller
% dhoeller@ethz.ch
%
% --
% Revision history
% [ 18.11.2019, NR ]    1.0
% [ 20.11.2019, DH ]    1.1

%% Clear workspace and command window
clear all;
close all;
clc;
%rng(1);

%% Options
% [M, N]
mapSize = [15, 20];
% Set to true to generate a random map of size mapSize, else set to false 
% to load the pre-exsisting example map
generateRandomWorld = true;

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
% IMPORTANT: Do not add or remove any global parameter in main.m
global GAMMA R Nc P_WIND L
GAMMA  = 0.2; % Shooter gamma factor
R = 2; % Shooter range
Nc = 10; % Time steps required to bring drone to base when it crashes
P_WIND = 0.1; % Gust of wind probability
L = 5; % Number of possible control inputs

% IDs of elements in the map matrix
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE 
FREE = 0;
TREE = 1;
SHOOTER = 2;
PICK_UP = 3;
DROP_OFF = 4;
BASE = 5;

% Index of each action in the P and G matrices. Use this ordering
global NORTH SOUTH EAST WEST HOVER
NORTH  = 1;
SOUTH = 2;
EAST = 3;
WEST = 4;
HOVER = 5;

%% Generate map
% map(m,n) represents the cell at indices (m,n) according to the axes
% specified in the PDF.
disp('Generate map');
if generateRandomWorld
	[map] = GenerateWorld(mapSize(1), mapSize(2));
else
    % We can load a pre-generated map.
    load('exampleWorld.mat');
end
MakePlots(map);
%% Generate state space
disp('Generate state space');
% Generate a (K x 3)-matrix 'stateSpace', where each accessible cell is
% represented by two rows (with and without carrying a package).
stateSpace = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) ~= TREE
            stateSpace = [stateSpace;
                          m, n, 0;
                          m, n, 1];
        end
    end
end
% State space size
global K
K=size(stateSpace,1);

%% Set the following to true as you progress with the files
transitionProbabilitiesImplemented = true;
stageCostsImplemented = true;
valueIterationImplemented = true; 
policyIterationImplemented = true;
linearProgrammingImplemented = true;

%% Compute the terminal state index
global TERMINAL_STATE_INDEX
if transitionProbabilitiesImplemented
    % TODO: Question a)
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end 

%% Compute base state index
global BASE_STATE_INDEX PICKUP_STATE_INDEX

BASE_STATE_INDEX = ComputeBaseStateIndex(stateSpace, map);
PICKUP_STATE_INDEX = ComputePickUpStateIndex(stateSpace, map);
%% Compute transition probabilities
if transitionProbabilitiesImplemented
    disp('Compute transition probabilities');
    % Compute the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.
    
    % TODO: Question b)
    P = ComputeTransitionProbabilities(stateSpace, map);
end

%% Compute stage costs
if stageCostsImplemented 
    disp('Compute stage costs');
    % Compute the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
    
    % TODO: Question c)
    G = ComputeStageCosts(stateSpace, map);
end

%% Solve stochastic shortest path problem
% Solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming
if valueIterationImplemented
    disp('Solve stochastic shortest path problem with Value Iteration');
    
    % TODO: Question d)
    [ J_opt_vi, u_opt_ind_vi ] = ValueIteration(P, G);
    
    if size(J_opt_vi,1)~=K || size(u_opt_ind_vi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if policyIterationImplemented
    disp('Solve stochastic shortest path problem with Policy Iteration');
    
    % TODO: Question d)
    [ J_opt_pi, u_opt_ind_pi ] = PolicyIteration(P, G);
    
    if size(J_opt_pi,1)~=K || size(u_opt_ind_pi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if linearProgrammingImplemented
    disp('Solve stochastic shortest path problem with Linear Programming');
    
    % TODO: Question d)
    [ J_opt_lp, u_opt_ind_lp ] = LinearProgramming(P, G);
    
    if size(J_opt_lp,1)~=K || size(u_opt_ind_lp,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end

%% Plot results (old)
% disp('Plot results');
% if valueIterationImplemented
%     MakePlots(map, stateSpace, J_opt_vi, u_opt_ind_vi, 'Value iteration');
% end
% if policyIterationImplemented
%     MakePlots(map, stateSpace, J_opt_pi, u_opt_ind_pi, 'Policy iteration');
% end
% if linearProgrammingImplemented
%     MakePlots(map, stateSpace, J_opt_lp, u_opt_ind_lp, 'Linear programming');
% end

%% Plot results Value Iteration
% disp('Plot results value iteration');
% if valueIterationImplemented
%     plotOptimalSolution(map, stateSpace, u_opt_ind_vi);
% end

%% Plot results Policy Iteration
% disp('Plot results policy iteration');
% if policyIterationImplemented
%     plotOptimalSolution(map, stateSpace, u_opt_ind_pi);
% end

%% Plot results Linear Programming
% disp('Plot results linear programming');
% if linearProgrammingImplemented
%     plotOptimalSolution(map, stateSpace, u_opt_ind_lp);
% end


%% RL stuff

%% Take optimal policy for the expert and sample
u_expert = u_opt_ind_pi;

% Sampling from optimal solition to learn Sarsa
T=100; %Number of trajectories

traj = SampleTrajMDP(P,u_opt_ind_pi,T);

%simulation(map,traj{1,6},stateSpace); % Uncomment to see simulation of a
%trajectory
Xtr=[];
for t=1:length(traj)
    Xtr=[Xtr; traj{1,t}' traj{2,t}'];
end

%% Hyperparameters Learning algorithms

T=5000; % Number of episodes for training
epsilon=0.01; % epsilon for eps-greedy policy
gamma=0.999; % discount factor
alpha=0.01; % step-size learning
steps=500; % max number of steps per episode
expert_initialization = false; % initialization using optimal policy info

%% SARSA from experts
initQ=ones(K, L);

if (expert_initialization)
    for i=1:length(Xtr)
        initQ(Xtr(i,1),Xtr(i,2))=10;
    end
end

disp('running SARSA')
[Q,reward_exp,n_steps_valid,cum_reward_valid,tot_n_valid] = SARSA(P,initQ,epsilon,gamma,alpha,T,steps);
[~,u] = (max(Q'));
u_Sarsa_exp=u';
disp('done')

% filtering metrics
reward_exp_filt = zeros(1,length(reward_exp));
reward_exp_filt(1) = reward_exp(1);
for i=2:length(reward_exp_filt)
    reward_exp_filt(i)=reward_exp_filt(i-1)*0.9+reward_exp(i)*0.1;
end

figure()
plot(1:T,reward_exp)
hold on
plot(1:T,reward_exp_filt,'LineWidth',2)
ylabel('reward')
xlabel('iter')
title('reward SARSA during learning')

% filtering metrics
n_steps_valid_filt = zeros(1,length(n_steps_valid));
n_steps_valid_filt(1) = n_steps_valid(1);
for i=2:length(n_steps_valid_filt)
    n_steps_valid_filt(i)=n_steps_valid_filt(i-1)*0.9+n_steps_valid(i)*0.1;
end

cum_reward_valid_filt = zeros(1,length(cum_reward_valid));
cum_reward_valid_filt(1) = cum_reward_valid(1);
for i=2:length(cum_reward_valid_filt)
    cum_reward_valid_filt(i)=cum_reward_valid_filt(i-1)*0.9+cum_reward_valid(i)*0.1;
end

figure()
subplot(2,1,1)
plot(1:tot_n_valid,n_steps_valid)
hold on
plot(1:tot_n_valid,n_steps_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('n_steps')
title('SARSA')
subplot(2,1,2)
plot(1:tot_n_valid,cum_reward_valid)
hold on
plot(1:tot_n_valid,cum_reward_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('cum_reward')


%plotOptimalSolution(map,stateSpace,u_Sarsa_exp);

%% SARSA with UCB from experts
initQ=ones(K, L);

if (expert_initialization)
    for i=1:length(Xtr)
        initQ(Xtr(i,1),Xtr(i,2))=10;
    end
end

disp('running SARSA UCB')
[Q,reward_exp,n_steps_valid,cum_reward_valid,tot_n_valid] = SARSA_UCB(P,initQ,gamma,alpha,T,steps);
[~,u] = (max(Q'));
u_SarsaUCB_exp=u';
disp('done')

% filtering metrics
reward_exp_filt = zeros(1,length(reward_exp));
reward_exp_filt(1) = reward_exp(1);
for i=2:length(reward_exp_filt)
    reward_exp_filt(i)=reward_exp_filt(i-1)*0.9+reward_exp(i)*0.1;
end

figure()
plot(1:T,reward_exp)
hold on
plot(1:T,reward_exp_filt,'LineWidth',2)
ylabel('reward')
xlabel('iter')
title('reward SARSA UCB during learning')

% filtering metrics
n_steps_valid_filt = zeros(1,length(n_steps_valid));
n_steps_valid_filt(1) = n_steps_valid(1);
for i=2:length(n_steps_valid_filt)
    n_steps_valid_filt(i)=n_steps_valid_filt(i-1)*0.9+n_steps_valid(i)*0.1;
end

cum_reward_valid_filt = zeros(1,length(cum_reward_valid));
cum_reward_valid_filt(1) = cum_reward_valid(1);
for i=2:length(cum_reward_valid_filt)
    cum_reward_valid_filt(i)=cum_reward_valid_filt(i-1)*0.9+cum_reward_valid(i)*0.1;
end

figure()
subplot(2,1,1)
plot(1:tot_n_valid,n_steps_valid)
hold on
plot(1:tot_n_valid,n_steps_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('n_steps')
title('SARSA UCB')
subplot(2,1,2)
plot(1:tot_n_valid,cum_reward_valid)
hold on
plot(1:tot_n_valid,cum_reward_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('cum_reward')


%plotOptimalSolution(map,stateSpace,u_SarsaUCB_exp);

%% Q-learning from experts

initQ=ones(K, L);

if (expert_initialization)
    for i=1:length(Xtr)
        initQ(Xtr(i,1),Xtr(i,2))=2;
    end
end

disp('running Q-Learning')
[Q,reward_exp,n_steps_valid,cum_reward_valid,tot_n_valid] = Q_Learning(P,initQ,epsilon,gamma,alpha,T,steps);
[~,u] = (max(Q'));
u_QLearning_exp=u';
disp('done')

% filtering metrics
reward_exp_filt = zeros(1,length(reward_exp));
reward_exp_filt(1) = reward_exp(1);
for i=2:length(reward_exp_filt)
    reward_exp_filt(i)=reward_exp_filt(i-1)*0.9+reward_exp(i)*0.1;
end

figure()
plot(1:T,reward_exp)
hold on
plot(1:T,reward_exp_filt,'LineWidth',2)
ylabel('reward')
xlabel('iter')
title('reward Q-Learning during learning')

% filtering metrics
n_steps_valid_filt = zeros(1,length(n_steps_valid));
n_steps_valid_filt(1) = n_steps_valid(1);
for i=2:length(n_steps_valid_filt)
    n_steps_valid_filt(i)=n_steps_valid_filt(i-1)*0.9+n_steps_valid(i)*0.1;
end

cum_reward_valid_filt = zeros(1,length(cum_reward_valid));
cum_reward_valid_filt(1) = cum_reward_valid(1);
for i=2:length(cum_reward_valid_filt)
    cum_reward_valid_filt(i)=cum_reward_valid_filt(i-1)*0.9+cum_reward_valid(i)*0.1;
end

figure()
subplot(2,1,1)
plot(1:tot_n_valid,n_steps_valid)
hold on
plot(1:tot_n_valid,n_steps_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('n_steps')
title('Q-Learning')
subplot(2,1,2)
plot(1:tot_n_valid,cum_reward_valid)
hold on
plot(1:tot_n_valid,cum_reward_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('cum_reward')

% plotOptimalSolution(map,stateSpace,u_QLearning_exp);

%% Q-learning with UCB from experts

initQ=ones(K, L);

if (expert_initialization)
    for i=1:length(Xtr)
        initQ(Xtr(i,1),Xtr(i,2))=2;
    end
end

disp('running Q-Learning UCB')
[Q,reward_exp,n_steps_valid,cum_reward_valid,tot_n_valid] = Q_Learning_UCB(P,initQ,gamma,alpha,T,steps);
[~,u] = (max(Q'));
u_QLearningUCB_exp=u';
disp('done')

% filtering metrics
reward_exp_filt = zeros(1,length(reward_exp));
reward_exp_filt(1) = reward_exp(1);
for i=2:length(reward_exp_filt)
    reward_exp_filt(i)=reward_exp_filt(i-1)*0.9+reward_exp(i)*0.1;
end

figure()
plot(1:T,reward_exp)
hold on
plot(1:T,reward_exp_filt,'LineWidth',2)
ylabel('reward')
xlabel('iter')
title('reward Q-Learning UCB during learning')

% filtering metrics
n_steps_valid_filt = zeros(1,length(n_steps_valid));
n_steps_valid_filt(1) = n_steps_valid(1);
for i=2:length(n_steps_valid_filt)
    n_steps_valid_filt(i)=n_steps_valid_filt(i-1)*0.9+n_steps_valid(i)*0.1;
end

cum_reward_valid_filt = zeros(1,length(cum_reward_valid));
cum_reward_valid_filt(1) = cum_reward_valid(1);
for i=2:length(cum_reward_valid_filt)
    cum_reward_valid_filt(i)=cum_reward_valid_filt(i-1)*0.9+cum_reward_valid(i)*0.1;
end

figure()
subplot(2,1,1)
plot(1:tot_n_valid,n_steps_valid)
hold on
plot(1:tot_n_valid,n_steps_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('n_steps')
title('Q-Learning UCB')
subplot(2,1,2)
plot(1:tot_n_valid,cum_reward_valid)
hold on
plot(1:tot_n_valid,cum_reward_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('cum_reward')

% plotOptimalSolution(map,stateSpace,u_QLearning_exp);

%% Double-Q-learning from experts

initQ1=ones(K, L);
initQ2=3*ones(K, L);

if (expert_initialization)
    for i=1:length(Xtr)
        initQ1(Xtr(i,1),Xtr(i,2))=2;
        initQ2(Xtr(i,1),Xtr(i,2))=4;
    end
end

disp('running Double Q-Learning')
[Q1,Q2,reward_exp,n_steps_valid,cum_reward_valid,tot_n_valid] = Double_Q_Learning(P,initQ1,initQ2,epsilon,gamma,alpha,T,steps);
Q=(Q1 + Q2)/2 ;
[~,u] = (max(Q'));
u_QLearning_exp=u';
disp('done')

% filtering metrics
reward_exp_filt = zeros(1,length(reward_exp));
reward_exp_filt(1) = reward_exp(1);
for i=2:length(reward_exp_filt)
    reward_exp_filt(i)=reward_exp_filt(i-1)*0.9+reward_exp(i)*0.1;
end

figure()
plot(1:T,reward_exp)
hold on
plot(1:T,reward_exp_filt,'LineWidth',2)
ylabel('reward')
xlabel('iter')
title('reward Double-Q-Learning during learning')

% filtering metrics
n_steps_valid_filt = zeros(1,length(n_steps_valid));
n_steps_valid_filt(1) = n_steps_valid(1);
for i=2:length(n_steps_valid_filt)
    n_steps_valid_filt(i)=n_steps_valid_filt(i-1)*0.9+n_steps_valid(i)*0.1;
end

cum_reward_valid_filt = zeros(1,length(cum_reward_valid));
cum_reward_valid_filt(1) = cum_reward_valid(1);
for i=2:length(cum_reward_valid_filt)
    cum_reward_valid_filt(i)=cum_reward_valid_filt(i-1)*0.9+cum_reward_valid(i)*0.1;
end

figure()
subplot(2,1,1)
plot(1:tot_n_valid,n_steps_valid)
hold on
plot(1:tot_n_valid,n_steps_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('n_steps')
title('Double-Q-Learning')
subplot(2,1,2)
plot(1:tot_n_valid,cum_reward_valid)
hold on
plot(1:tot_n_valid,cum_reward_valid_filt,'LineWidth',2)
xlabel('n_validation')
ylabel('cum_reward')

% plotOptimalSolution(map,stateSpace,u_QLearning_exp);


