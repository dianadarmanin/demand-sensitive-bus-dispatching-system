import os, sys
import time
import traci
import sumolib
import gym
import argparse
from playsound import playsound

import PerformanceHelper
from env import SumoEnvironment
import gym
import random

from d3rlpy.algos import DQN
from d3rlpy.online.buffers import ReplayBuffer
from d3rlpy.online.explorers import LinearDecayEpsilonGreedy
from d3rlpy.online.iterators import train_single_env

import NoControl
import RuleBasedControl

control_strategy = "RBC"
use_gui = False
performance = {'NC': dict(),
               'RBC': dict(),
               'RLBC': dict()}
net_file = 'LargeScale/Delgado/network_file.net.xml'
route_file = 'LargeScale/Delgado/demand_file.rou.xml'
config_file = 'LargeScale/Delgado/configuration.sumocfg'


def start_sumo():
    """
    Starts Sumo Simulation
    """
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    if use_gui:
        sumo_binary = sumolib.checkBinary('sumo-gui')
    else:
        sumo_binary = sumolib.checkBinary('sumo')

    sumoCmd = [sumo_binary, "-c", config_file, "--start", "--no-warnings", "--quit-on-end"]
    traci.start(sumoCmd)
    # traci.gui.setSchema('View #0', 'real world')


if control_strategy == "NC":
    start_sumo()
    TOTAL_BUSSES = 14
    bus_stops = traci.busstop.getIDList()
    tot_network_distance = 500 * traci.edge.getIDCount()
    edges_ids = [int(e.split("E")[1]) for e in traci.edge.getIDList() if "E" in e]
    edges_ids.sort()

    last_edge = int(edges_ids[-1])
    first_edge = int(edges_ids[0])
    cycles = dict()
    cycle_history = dict()  # todo change in others
    distance_travelled = dict()
    distances_per_t_steps = dict()
    metrics = {'Headway Variance': [],
               'Average Wait Time Variance': []
               }
    total_dist_travelled = dict()
    # total_dist_travelled_last = dict()
    performance['NC'] = {'Time': [],
                         'Headways': [],
                         'Headway Variance': [],
                         'Average Wait Time Variance': [],
                         'Total Distances Travelled': []}
    distances_info = {'Time': [],
                      'Distances': []}
    busses_distance_travelled = dict()
    headways = dict()
    passengers_at_all_stops = []
    loops_done = False
    continue_flag = False
    # while traci.simulation.getMinExpectedNumber() > 0:
    while traci.simulation.getTime() < 5500:
        # if 'vehicle_0' not in list(cycles.keys()):
        #     pass
        # else:
        #     if continue_flag:
        #         pass
        #     else:
        #         print("[INFO] Looping Till Cycle 6 is reached")
        #         while cycles['vehicle_0'] < 1:
        #             for bus in traci.vehicle.getIDList():
        #                 if bus not in list(cycle_history.keys()):
        #                     cycle_history[bus] = [-1,
        #                                           0]  # list (past_edge, current_edge) for vehicle_0 to keep track of history #todo
        #                 if bus not in list(cycles.keys()):
        #                     cycles[bus] = 0  # todo
        #             cycle_history, cycles = NoControl.update_cycles(traci.vehicle.getIDList(), cycle_history, cycles,
        #                                                             first_edge, last_edge)
        #             traci.simulationStep()
        #         loops_done = True
        #
        # if loops_done:
        #     continue_flag = True
        #     start_time = traci.simulation.getTime()
        start_time = 0

        traci.simulationStep()
        curr_time = traci.simulation.getTime()
        active_busses = traci.vehicle.getIDList()  # list of active busses in network
        for active_bus in active_busses:
            traci.vehicle.highlight(active_bus)  # for sumo-gui visualisation purposes
            distance_travelled[active_bus] = NoControl.get_distance_on_net(active_bus)
            if active_bus not in list(cycle_history.keys()):
                cycle_history[active_bus] = [-1,
                                             0]  # list (past_edge, current_edge) for vehicle_0 to keep track of history #todo
            if active_bus not in list(cycles.keys()):
                cycles[active_bus] = 0  # todo

        # create a dictionary of busses sorted according to the bus which is the closest to the ent
        busses_distance_travelled = dict(sorted(distance_travelled.items(), key=lambda item: item[1], reverse=True))
        headways = NoControl.get_headways(active_busses, busses_distance_travelled, headways,
                                          TOTAL_BUSSES)  # front headway for each vehicle
        cycle_history, cycles = NoControl.update_cycles(active_busses, cycle_history, cycles, first_edge, last_edge)

        # -------------------------------------------------------
        if curr_time % 50 == 0:
            PerformanceHelper.progress_print(curr_time, active_busses, start_time, 5500)
            performance['NC'], total_dist_travelled = NoControl.update_performance(performance['NC'], curr_time,
                                                                                   active_busses, headways,
                                                                                   bus_stops,
                                                                                   total_dist_travelled,
                                                                                   distance_travelled, cycles,
                                                                                   tot_network_distance)

        total_dist_travelled = NoControl.get_total_distances(active_busses, total_dist_travelled, distance_travelled,
                                                             cycles,
                                                             tot_network_distance)
        for bus in active_busses:
            distances_per_t_steps[bus] = total_dist_travelled[bus]
        if curr_time % 50 == 0:
            print(distances_per_t_steps)
        distances_info['Time'].append(curr_time)
        distances_info['Distances'].append(distances_per_t_steps.copy())

        persons = []
        for stop in traci.busstop.getIDList():
            persons.append(traci.busstop.getPersonCount(stop))
        passengers_at_all_stops.append(persons)
    sys.stdout.flush()
    traci.close()

elif control_strategy == "RBC":
    start_sumo()
    TOTAL_BUSSES = 14
    bus_stops = traci.busstop.getIDList()
    edges_ids = [int(e.split("E")[1]) for e in traci.edge.getIDList() if "E" in e]
    edges_ids.sort()
    tot_network_distance = 500 * traci.edge.getIDCount()

    last_edge = int(edges_ids[-1])
    first_edge = int(edges_ids[0])
    cycles = {'vehicle_0': 0}
    cycle_history = dict()  # todo change in others
    distance_travelled = dict()
    vehs_who_are_stopped = dict()
    total_dist_travelled = dict()
    distances_per_t_steps = dict()
    headways = dict()

    performance['RBC'] = {'Time': [],
                          'Headways': [],
                          'Headway Variance': [],
                          'Average Wait Time Variance': [],
                          'Total Distances Travelled': []}
    distances_info = {'Time': [],
                      'Distances': []}
    actions = {'stop_and_waits':0, 'limit_boardings':0, 'skip_stops':0}
    last_action = [None,None]
    # while traci.simulation.getMinExpectedNumber() > 0:
    while traci.simulation.getTime() < 5500:
        traci.simulationStep()

        # -------------------------------------------------------
        # ------------ UPDATE NECESSARY VARIABLES ---------------
        # -------------------------------------------------------
        curr_time = traci.simulation.getTime()
        active_busses = traci.vehicle.getIDList()  # list of active busses in network
        active_persons = traci.person.getIDList()  # list of active people in network
        stop_locations = RuleBasedControl.get_bus_stop_locations()  # dict of bus stops and respective lane IDs

        for active_bus in active_busses:
            traci.vehicle.highlight(active_bus)  # for sumo-gui visualisation purposes
            distance_travelled[active_bus] = RuleBasedControl.get_distance_on_net(active_bus)
            if active_bus not in list(cycle_history.keys()):
                cycle_history[active_bus] = [-1, 0]
            if active_bus not in list(cycles.keys()):
                cycles[active_bus] = 0  # todo

            if active_bus not in list(vehs_who_are_stopped.keys()):
                vehs_who_are_stopped[active_bus] = [False, False]

        # create a dictionary of busses sorted according to the bus which is the closest to the ent
        busses_distance_travelled = dict(sorted(distance_travelled.items(), key=lambda item: item[1], reverse=True))

        passengers_at_each_stop = RuleBasedControl.get_passengers_waiting(
            bus_stops)  # no of passengers waiting at each stop
        vehs_at_each_stop = RuleBasedControl.get_vehs_at_stops(
            bus_stops)  # vehicles waiting at each stop - dict of bus stops
        vehs_who_are_stopped = RuleBasedControl.get_vehs_who_are_stopped(active_busses,
                                                                         vehs_who_are_stopped)  # vehicles who are currently stopped - dict of busses (bool)
        headways = RuleBasedControl.get_headways(active_busses, headways, busses_distance_travelled,
                                                 TOTAL_BUSSES)  # front headway for each vehicle
        passengers = RuleBasedControl.get_passengers()  # no of passengers in each vehicle
        persons_in_vehs = RuleBasedControl.get_persons_in_vehs(active_busses)  # list of persons in each vehicle
        capacities = RuleBasedControl.get_veh_capacity(active_busses)  # vehicle capacity
        vehicle_waiting_time = RuleBasedControl.get_waiting_time(
            active_busses)  # how long each vehicle has been waiting behind a bus that is stopped
        # -------------------------------------------------------

        # print(traci.simulation.getTime(), "     ", busses_distance_travelled)
        if curr_time % 50 == 0:
            print(curr_time, "\t [",len(active_busses),"]", "Headway Variance: ", int(RuleBasedControl.headway_variability_reward(headways)),
                  "Average Wait Time Variance :", int(RuleBasedControl.waiting_time_variability_reward(bus_stops)))
        if len(active_busses) == TOTAL_BUSSES:  # once at least one cycle has been made
            if curr_time % 50 == 0:
                shuffled_active = list(active_busses).copy()
                random.shuffle(shuffled_active)
                for bus in shuffled_active:
                    road_id, road_idx = RuleBasedControl.get_road_from_lane(traci.vehicle.getLaneID(bus))
                    upcoming_stop = RuleBasedControl.get_upcoming_stop(road_idx, stop_locations)

                    if vehs_who_are_stopped[bus][0] is True and vehs_who_are_stopped[bus][1] is False:
                        current_stop = RuleBasedControl.get_current_stop(bus, vehs_at_each_stop)
                        follower = RuleBasedControl.get_follower(busses_distance_travelled, bus)
                        # LIMIT BOARDING
                        if (passengers[bus] / capacities[bus] > 0.6) and (
                                passengers[follower] / capacities[follower] < 0.5):
                            if last_action[0] == 1 and last_action[1] == bus:
                                pass
                            else:
                                actions['limit_boardings'] += 1
                                print("[@{}] Limiting Boarding of {} from {} due to bus capacity limit reached and "
                                      "follower has space".format(curr_time, bus, current_stop))
                                traci.vehicle.resume(bus)  # skip stop
                                last_action = [1, bus]
                                break
                        # SKIP STOP
                        if passengers_at_each_stop[current_stop] * 1.5 <= passengers_at_each_stop[upcoming_stop]:
                            if last_action[0] == 2 and last_action[1] == bus:
                                pass
                            else:
                                actions['skip_stops'] += 1
                                print(
                                    "[@{}] Skip Stop instruction to {} from {} due to no passengers at stop".format(curr_time,
                                                                                                                 bus,
                                                                                                                 current_stop))
                                traci.vehicle.resume(bus)  # skip stop
                                last_action = [2, bus]
                                break

                    # STOP AND WAIT
                    if headways[bus] < 12:
                        if last_action[0] == 0 and last_action[1] == bus:
                            pass
                        else:
                            actions['stop_and_waits'] += 1
                            print("[@{}] Inserting Stop of [200s] at {} for bus {}".format(curr_time,
                                                                                           upcoming_stop, bus))
                            traci.vehicle.setStop(bus, stop_locations[upcoming_stop][0], pos=30, laneIndex=1,
                                                  duration=30)
                            last_action = [0, bus]
                            break
        cycle_history, cycles = RuleBasedControl.update_cycles(active_busses, cycle_history, cycles, first_edge,
                                                               last_edge)
        if curr_time % 50 == 0:
            performance['RBC'] = RuleBasedControl.update_performance(performance['RBC'], curr_time, active_busses, headways,
                                                                     bus_stops,
                                                                     total_dist_travelled,
                                                                     distance_travelled, cycles,
                                                                     tot_network_distance)
        total_dist_travelled = NoControl.get_total_distances(active_busses, total_dist_travelled, distance_travelled,
                                                             cycles,
                                                             tot_network_distance)
        for bus in active_busses:
            distances_per_t_steps[bus] = total_dist_travelled[bus]
        # if curr_time % 50 == 0:
        #     print(distances_per_t_steps)
        distances_info['Time'].append(curr_time)
        distances_info['Distances'].append(distances_per_t_steps.copy())

    sys.stdout.flush()
    traci.close()

elif control_strategy == "RLBC":

    env = SumoEnvironment(net_file=net_file,
                          route_file=route_file,
                          config_file=config_file,
                          use_gui=False)
    eval_env = SumoEnvironment(net_file=net_file,
                               route_file=route_file,
                               config_file=config_file,
                               use_gui=False)

    # batch_size = 32,

    # setup algorithm
    dqn = DQN(n_epochs=30,
              batch_size=32,
              learning_rate=2.5e-4,
              target_update_interval=100,
              use_gpu=True)

    # setup replay buffer
    buffer = ReplayBuffer(maxlen=1000000, env=env)

    # setup explorers
    explorer = LinearDecayEpsilonGreedy(start_epsilon=1.0,
                                        end_epsilon=0.1,
                                        duration=10000)

    # start training
    dqn.fit_online(env,
                   buffer,
                   explorer=explorer,  # you don't need this with probablistic policy algorithms
                   eval_env=eval_env,
                   n_steps=30000,  # the number of total steps to train.
                   timelimit_aware=False,
                   n_steps_per_epoch=1000,
                   update_interval=10)  # update parameters every 10 steps.
    dqn.save_model("dqn.pt")
    dqn.save_policy('policy.pt')
    performance['RLBC'] = env.performance

else:
    print("[ERROR] Control Strategy provided does not exist!")

if control_strategy == 'NC':
    distances_info['Time'] = distances_info['Time'][1:]
    distances_info['Distances'] = distances_info['Distances'][1:]
    PerformanceHelper.plot_dist_time_graph_spec(distances_info, control_strategy)
    PerformanceHelper.plot_reward1_time_graph(performance['NC'], control_strategy)
    PerformanceHelper.plot_reward2_time_graph(performance['NC'], control_strategy)

    i = 0
    for entry in passengers_at_all_stops:
        print(i, "\t", entry)
        i += 1
elif control_strategy == 'RBC':
    distances_info['Time'] = distances_info['Time'][1:]
    distances_info['Distances'] = distances_info['Distances'][1:]
    PerformanceHelper.plot_dist_time_graph_spec(distances_info, control_strategy)
    PerformanceHelper.plot_reward1_time_graph(performance['RBC'], control_strategy)
    PerformanceHelper.plot_reward2_time_graph(performance['RBC'], control_strategy)
    print("Actions taken")
    print(actions)
