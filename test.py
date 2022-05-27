import gym

import argparse
import os
import sys

import PerformanceHelper
from env import SumoEnvironment
import gym

from d3rlpy.algos import DQN
from d3rlpy.online.buffers import ReplayBuffer
from d3rlpy.online.explorers import LinearDecayEpsilonGreedy
from d3rlpy.online.iterators import train_single_env

net_file = 'LargeScale/Delgado/network_file.net.xml'
route_file = 'LargeScale/Delgado/demand_file.rou.xml'
config_file = 'LargeScale/Delgado/configuration.sumocfg'

env = SumoEnvironment(net_file=net_file,
                      route_file=route_file,
                      config_file=config_file,
                      use_gui=False, name="Env")
eval_env = SumoEnvironment(net_file=net_file,
                           route_file=route_file,
                           config_file=config_file,
                           use_gui=False, name="Eval Env")

# batch_size = 32,

# setup algorithm
dqn = DQN(batch_size=32,
          learning_rate=2.5e-4,
          target_update_interval=25,
          use_gpu=True)

# setup replay buffer
# todo tista tamlu kbir max len
buffer = ReplayBuffer(maxlen=20000, env=env)

# setup explorers
explorer = LinearDecayEpsilonGreedy(start_epsilon=1.0,
                                    end_epsilon=0.1,
                                    duration=100)

# start training
dqn.fit_online(env,
               buffer,
               explorer=explorer,  # you don't need this with probablistic policy algorithms
               eval_env=eval_env,
               n_steps=50,  # the number of total steps to train.
               timelimit_aware=False,
               n_steps_per_epoch=1,
               update_interval=3)  # update parameters every 10 steps.

dqn.save_model("dqn.pt")
dqn.save_policy('policy.pt')
distances2 = eval_env.distances_info
performance = env.performance
performance_rl = env.rl_performance
performance2 = eval_env.performance
performance_rl2 = eval_env.rl_performance

PerformanceHelper.plot_dist_time_graph(distances2, "RLBC2")
PerformanceHelper.plot_reward1_time_graph(performance, "RLBC")
PerformanceHelper.plot_reward2_time_graph(performance, "RLBC")
PerformanceHelper.plot_cost_steps_graph(performance_rl, " RLBC1")
PerformanceHelper.plot_reward1_time_graph(performance2, "RLBC2")
PerformanceHelper.plot_reward2_time_graph(performance2, "RLBC2")
PerformanceHelper.plot_cost_steps_graph(performance_rl2,"RLBC2")

print("Actions taken by env")
print(env.actions_taken)
print("Actions taken by eval env")
print(eval_env.actions_taken)
# todo performance
# Sutton page132
