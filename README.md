# DPOC-Project
Programming Exercise part of the course Dynamic Programming and Optimal Control, ETH Zurich, academic year 2019/2020. Read ProgrammingExercise.pdf for problem and scripts description. The aim of this programming exercise is to solve a stochastic shortest path problem using Value Iteration, Policy Iteration and Linear Programming. The scripts coded by the student are:

1. ComputeTerminalStateIndex.m
2. ComputeTransitionProbabilities.m
3. ComputeStageCosts.m
4. PolicyIteration.m
5. ValueIteration.m
6. LinearProgramming.m

Run main.m for checking the solution obtained.

# RL extension
I have extended the programming exercise by solving the same stochastic shortest path problem using Reinforcement Learning (RL) algorithms:

1. SARSA w and w/o initialization from expert
2. Q-Learning w and w/o initialization from expert
3. Double-Q-Learning w and w/o initialization from expert

Where the initialization is guided by the expert, some trajectories are sampled using the optimal policy obtained through Dynamic Programming, and the Q-values of the state-action pairs visited are initialized at a higher value. The functions that implement SARSA, Q-Learning and Double-Q-Learning can be found in scripts SARSA.m, Q_Learning.m and Double_Q_Learning.m, respectively. In these, an epsilon-greedy policy is used for exploration. SARSA and Q-Learning are implemented also using Upper Confidence Bounds (UCB) for exploration (in SARSA_UCB.m and Q_Learning_UCB.m, respectively).
