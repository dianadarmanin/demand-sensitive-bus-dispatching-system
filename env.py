import os
import sys
import numpy
import matplotlib.pyplot as plt
from PerformanceHelper import *
import traci
import sumolib
from gym import Env
# import traci.constants as tc
from gym import spaces
# from ray.rllib.env.multi_agent_env import MultiAgentEnv
import numpy as np
import pandas as pd

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")


def get_bus_stop_locations():
    """
    :return: bus_stop_locations: Dictionary of bus stops and their respective lane_id
    :rtype: dict()
    """
    # returns a dict of bus stops whose values are tuples with (road id, road index): example (gneE4, 4)
    bus_stop_locations = dict()
    for bus_stop in traci.busstop.getIDList():
        lane_id = traci.busstop.getLaneID(bus_stop)
        bus_stop_locations[bus_stop] = get_road_from_lane(lane_id)
    return bus_stop_locations


def get_road_from_lane(lane_id):
    """
    :param lane_id: lane_ids are in the format of gneE0_1, gneE0_0
    :type lane_id:
    :return: road_id: road_ids are in the format of gneE0, gneE1...
    :return: road_idx: the number associated with the road
    :rtype:
    """
    if "E" in lane_id:  # its an edge because it contains an E
        road_id = lane_id.split("_")[0]
        road_idx = road_id.split("E")[1]
        return road_id, road_idx
    elif "J" in lane_id:  # either junction "gneJ0" or ":gneJ0_0"
        if lane_id[0] != ":":
            road_idx = lane_id.split("J")[1]
        else:
            road_idx_0 = lane_id.split("J")[1]
            road_idx = road_idx_0.split("_")[0]
        road_id = "gneE" + road_idx

    return road_id, road_idx


def get_distance_on_net(veh_id):
    """
    Returns total distance travelled from edge 0 pos 0 to current edge current position
    :param veh_id: bus
    :return: dist_on_network: distance travelled
    """
    dist_on_network = None
    lane_pos = traci.vehicle.getLanePosition(veh_id)
    road_id = traci.vehicle.getRoadID(veh_id)
    if "E" in road_id:  # it's an edge because it contains an E
        road_idx = road_id.split("E")[1]
        dist_on_network = lane_pos + (500 * int(road_idx))
    elif "J" in road_id:  # either junction "gneJ0" or ":gneJ0_0"
        if road_id[0] != ":":
            road_idx = road_id.split("J")[1]
        else:
            road_idx_0 = road_id.split("J")[1]
            road_idx = road_idx_0.split("_")[0]
        road_id = "gneE" + road_idx
        dist_on_network = 500 * int(road_idx)
    return dist_on_network


def one_hot_encode(data_list, variable):
    """
    One hot encodes a list of data
    """
    ohe = []
    for item in data_list:
        if variable == item:
            ohe.append(1)
        else:
            ohe.append(0)
    return ohe


class SumoEnvironment:
    """
    SUMO Environment for Bus Control
    """

    def __init__(self, net_file, route_file, config_file, out_csv_name=None, use_gui=False, name=None):
        print("initialising new sumo environment for ", name)
        self.name = name

        self._net = net_file
        self._route = route_file
        self._config = config_file
        self.use_gui = use_gui
        if self.use_gui:
            self._sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            self._sumo_binary = sumolib.checkBinary('sumo')

        # traci.start([sumolib.checkBinary('sumo'), '-c', self._config])  # start only to retrieve information
        sumo_cmd = [self._sumo_binary, "-c", self._config, "--no-warnings", "--quit-on-end"]
        if self.use_gui:
            sumo_cmd.append('--start')
        traci.start(sumo_cmd)

        # Network Variables
        self.tot_network_distance = 500 * traci.edge.getIDCount()
        self.total_busses = 14
        edges_ids = [int(e.split("E")[1]) for e in traci.edge.getIDList() if "E" in e]
        edges_ids.sort()
        self.last_edge = int(edges_ids[-1])
        self.first_edge = int(edges_ids[0])

        # Environment Variables
        #  busses
        self.active_busses = []
        self.capacities = dict()
        self.distance_travelled = dict()
        self.busses_distance_travelled = dict()
        self.total_distances_travelled = dict()
        self.vehs_who_are_stopped = dict()
        self.headways = dict()
        self.passengers = dict()
        self.capacities = dict()
        self.cycles = {'vehicle_0': 0}
        self.cycle_history = dict()
        #  bus stops
        self.bus_stops = traci.busstop.getIDList()
        self.stop_locations = get_bus_stop_locations()
        self.vehs_at_each_stop = dict()
        #  passengers
        self.average_wait_time_at_each_stop = dict()
        self.passengers_at_each_stop = dict()
        self.avg_waiting_time = dict()

        # Evaluation Variables
        self.current_total_distances_per_bus = dict()
        self.performance = {'Time': [],
                            'Headways': [],
                            'Headway Variance': [],
                            'Average Wait Time Variance': [],
                            'Total Distances Travelled': []}
        self.rl_performance = {'Step': [],
                               'Cost Function': []}
        self.distances_info = {'Step': [],
                               'Time': [],
                               'Distances': []}
        self.performance_history = []
        self.rl_performance_history = []
        self.actions_taken = {'Continue': 0, 'SkipStop': 0, 'StopAndWait': 0, 'LimitBoarding': 0}

        self.out_csv_name = out_csv_name
        # GYM Variables
        pass_high, pass_low = self.get_high_low_pass_p_stop()
        one_hot_encoded_stop_high, one_hot_encoded_stop_low = self.get_high_low_ohe_bus_stop()
        no_of_edges = traci.edge.getIDCount()
        high = np.array(
            pass_high + [3000, 3000, 3000, no_of_edges] + one_hot_encoded_stop_high + [no_of_edges, no_of_edges, 100,
                                                                                       100], dtype=np.float32)
        low = np.array(pass_low + [0, 0, 0, 0] + one_hot_encoded_stop_low + [0, 0, 0, 0], dtype=np.float32)

        self.action_space = spaces.Discrete(len(['StopAndWait', 'SkipStop', 'LimitBoarding', 'Continue']))
        self.reward_range = (-float('inf'), float(0))
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.restart_count = 0
        self.last_reward = None
        self.last_action = None
        self.last_decision_location = None
        self.last_decision_bus = None
        self.decisions_made = []
        self.rl_steps = 0
        traci.close()

    def reset(self):
        print("[INFO] ", self.name, " restarting ", self.restart_count)
        try:
            self.close()
        except traci.FatalTraCIError:
            pass

        sumo_cmd = [self._sumo_binary, "-c", self._config, "--no-warnings", "--quit-on-end"]
        if self.use_gui:
            sumo_cmd.append('--start')
        traci.start(sumo_cmd)

        """Restart important environment variables"""
        self._reset_variables()

        # loop to get some action going
        while self.cycles['vehicle_0'] == 0:
            self._sumo_step()

        ready_to_make_decision = False
        decisions = []
        while not ready_to_make_decision:  # while list is empty - no decisions to be made
            decisions = self._decision_check()
            if len(decisions) > 0:
                ready_to_make_decision = True
            else:
                self._sumo_step()
        # at this point it has been decided that a decision needs to be made

        decision_bus = decisions[0]

        # observe new state and reward
        observation = self._compute_observations(decision_bus)
        self.last_decision_bus = decision_bus
        self.restart_count += 1
        return observation

    def _reset_variables(self):
        #  busses
        self.active_busses = []
        self.capacities = dict()
        self.distance_travelled = dict()
        self.busses_distance_travelled = dict()
        self.total_distances_travelled = dict()
        self.vehs_who_are_stopped = dict()
        self.headways = dict()
        self.passengers = dict()
        self.capacities = dict()
        self.total_dist_travelled = dict()
        self.cycles = {'vehicle_0': 0}
        self.cycle_history = dict()
        #  bus stops
        self.bus_stops = traci.busstop.getIDList()
        self.stop_locations = get_bus_stop_locations()
        self.vehs_at_each_stop = dict()
        #  passengers
        self.average_wait_time_at_each_stop = dict()
        self.passengers_at_each_stop = dict()
        self.avg_waiting_time = dict()
        # Evaluation Variables
        self.current_total_distances_per_bus = dict()
        self.performance = {'Time': [],
                            'Headways': [],
                            'Headway Variance': [],
                            'Average Wait Time Variance': [],
                            'Total Distances Travelled': []}
        self.rl_performance = {'Step': [],
                               'Cost Function': []}
        self.distances_info = {'Step': [],
                               'Time': [],
                               'Distances': []}
        self.performance_history = []
        self.rl_performance_history = []
        self.decisions_made = []
        self.rl_steps = 0

    def step(self, action):
        """
        Does one step which executes action ,computes observation and reward
        """
        print("[INFO] ", self.name, " Step #", self.rl_steps)

        """Execute Action"""
        if action == 3:  # 'Continue'
            self.actions_taken['Continue'] += 1
            self._sumo_step()
        else:
            self._apply_action(action, self.last_decision_bus)

        self._sumo_step()  # need to do at least one step

        ready_to_make_decision = False
        decisions = []
        while not ready_to_make_decision:  # while list is empty
            decisions = self._decision_check()
            if len(decisions) > 0:
                ready_to_make_decision = True
            else:
                self._sumo_step()
        # at this point it has been decided that a decision needs to be made

        decision_bus = decisions[0]

        """observe new state and reward"""
        observation = self._compute_observations(decision_bus)
        reward = self._compute_rewards()
        if self.rl_steps > 50:
            done = True
        else:
            done = False
        info = {}
        self.last_reward = reward
        self.last_action = action
        self.last_decision_bus = decision_bus

        self._update_rl_performance()
        self.rl_steps += 1
        return observation, reward, done, info

    @property
    def sim_step(self):
        """
        Return current simulation second on SUMO
        """
        return traci.simulation.getTime()  # seconds

    def close(self):
        """
        Closes traci conneciton
        """
        traci.close()

    def _sumo_step(self):
        self._update_necessary_variables()
        if self.sim_step % 50 == 0:
            self._update_performance()
        self._update_distances()
        traci.simulationStep()

    def _compute_observations(self, bus):
        """
        Return the current observation for a bus
        """
        self._get_passengers_waiting()  # no of passengers waiting at each stop
        self._get_average_wait_times()
        self.get_passengers()  # no of passengers in each vehicle
        self._get_veh_capacity()  # vehicle capacity
        stop_passengers = list(self.passengers_at_each_stop.values())
        follower, leader = self._get_consecutive_busses(bus)

        h1 = self.headways[bus]
        h2 = self.headways[follower]
        h3 = self.headways[leader]
        road_id, road_idx = get_road_from_lane(traci.vehicle.getLaneID(bus))
        upcoming_stop = self._get_upcoming_stop(road_idx)
        upcoming_stop_one_hot_encoded = one_hot_encode(self.bus_stops, upcoming_stop)
        follower_road_id, follower_road_idx = get_road_from_lane(traci.vehicle.getLaneID(follower))
        leader_road_id, leader_road_idx = get_road_from_lane(traci.vehicle.getLaneID(leader))
        p1 = self.passengers[bus]  # number of passengers on bus
        p2 = self.passengers[follower]  # number of passengers on follower
        observations = stop_passengers + [h1, h2, h3, road_idx] + upcoming_stop_one_hot_encoded + [follower_road_idx,
                                                                                                   leader_road_idx, p1,
                                                                                                   p2]
        observations = numpy.array(observations, dtype='float32')
        return observations

    def _compute_rewards(self):
        reward = (1 * self._headway_variability_reward()) + (1 * self._waiting_time_variability_reward())
        return reward

    def _apply_action(self, action, bus):
        if action == 0:  # 'SkipStop'
            stop = list(self.vehs_at_each_stop.keys())[list(self.vehs_at_each_stop.values()).index(bus)]
            passengers = self.passengers_at_each_stop[stop]
            try:
                traci.vehicle.resume(bus)  # skip stop
                self.actions_taken['SkipStop'] += 1
            except traci.exceptions.TraCIException:
                print("Failed to resume from stopping for vehicle 'vehicle_1'", " stopping at ", stop, " with ",
                      passengers, " passengers")

        elif action == 1:  # 'StopAndWait'
            road_id, road_idx = get_road_from_lane(traci.vehicle.getLaneID(bus))
            upcoming_stop = self._get_upcoming_stop(road_idx)
            upcoming_stop_pos = traci.busstop.getEndPos(upcoming_stop)
            traci.vehicle.setStop(bus, self.stop_locations[upcoming_stop][0], pos=upcoming_stop_pos, laneIndex=1,
                                  duration=30)
            self.actions_taken['StopAndWait'] += 1

        elif action == 2:  # 'LimitBoarding'
            capacity_reached = False
            follower = self._get_follower(bus)

            while traci.vehicle.isAtBusStop(bus) and not capacity_reached:
                if (self.passengers[bus] / self.capacities[bus] > 0.75) and (
                        self.passengers[follower] / self.capacities[follower] < 0.5):
                    capacity_reached = True
                elif traci.vehicle.isAtBusStop(bus):
                    # print("Sumo Stepping until capacity reached")
                    self._sumo_step()
            if traci.vehicle.isAtBusStop(bus):
                traci.vehicle.resume(bus)  # leave stop
                self.actions_taken['LimitBoarding'] += 1

        self.decisions_made.append([self.sim_step, action, bus])

    def _decision_check(self):
        make_decision = []
        for bus in self.active_busses:
            if self.vehs_who_are_stopped[bus][0] is True and self.vehs_who_are_stopped[bus][1] is False:
                make_decision.append(bus)
        return make_decision

    def _headway_variability_reward(self):
        headways = self.headways
        if len(headways) == 0:
            return 0
        mean = sum(list(headways.values())) / len(headways)
        numerator = 0
        for headway in headways:
            numerator += ((headways[headway] - mean) ** 2)
        variance = numerator / len(headways)
        reward = - variance
        return reward

    def _waiting_time_variability_reward(self):
        bus_stops = self.bus_stops
        avg_waiting_time = dict()
        for bus_stop in bus_stops:
            people_at_stop = traci.busstop.getPersonIDs(bus_stop)
            no_of_ppl_at_stop = traci.busstop.getPersonCount(bus_stop)
            tot_waiting_time = 0
            avg_waiting_time[bus_stop] = 0
            if no_of_ppl_at_stop != 0:
                for person in people_at_stop:
                    tot_waiting_time += (traci.person.getWaitingTime(person) ** 2)
                avg_waiting_time[bus_stop] = tot_waiting_time / no_of_ppl_at_stop  # MEAN SQUARED ERROR

        mean = sum(list(avg_waiting_time.values())) / len(avg_waiting_time)
        numerator = 0
        for bus_stop in avg_waiting_time:
            numerator += ((avg_waiting_time[bus_stop] - mean) ** 2)
        variance = numerator / len(avg_waiting_time)
        reward = - variance
        return reward

    def _update_necessary_variables(self):
        newly_added_busses = [bus for bus in traci.vehicle.getIDList() if bus not in self.active_busses]
        self.active_busses.extend(newly_added_busses)
        self._update_cycles()
        for active_bus in self.active_busses:
            self.distance_travelled[active_bus] = get_distance_on_net(active_bus)
            if active_bus not in list(self.vehs_who_are_stopped.keys()):
                self.vehs_who_are_stopped[active_bus] = [False, False]
        self._get_vehs_at_stops()
        self.busses_distance_travelled = dict(
            sorted(self.distance_travelled.items(), key=lambda item: item[1], reverse=True))
        self._get_headways()  # front headway for each vehicle
        self._get_vehs_who_are_stopped()  # vehicles who are currently stopped - dict of busses (bool)

    def _update_cycles(self):
        for bus in self.active_busses:
            if bus not in list(self.cycle_history.keys()):
                self.cycle_history[bus] = [-1, 0]
            if bus not in list(self.cycles.keys()):
                self.cycles[bus] = 0

            road_id, road_idx = get_road_from_lane(traci.vehicle.getLaneID(bus))

            if self.cycle_history[bus][1] != road_idx:
                self.cycle_history[bus][0] = int(self.cycle_history[bus][1])
                self.cycle_history[bus][1] = int(road_idx)

            if self.cycle_history[bus][0] == self.last_edge and self.cycle_history[bus][1] == self.first_edge:
                self.cycles[bus] += 1
                if bus == 'vehicle_0':
                    print("Cycles completed by vehicle 0: ", self.cycles[bus])

    def _update_performance(self):
        self.performance['Time'].append(self.sim_step)
        self.performance['Headways'].append(self.headways)
        self.performance['Headway Variance'].append(self._headway_variability_reward())
        self.performance['Average Wait Time Variance'].append(self._waiting_time_variability_reward())
        self.performance['Total Distances Travelled'].append(self._get_total_distances())

    def _update_distances(self):
        total_distances_travelled = self._get_total_distances()
        for bus in self.active_busses:
            self.current_total_distances_per_bus[bus] = total_distances_travelled[bus]
        self.distances_info['Step'].append(self.rl_steps)
        self.distances_info['Time'].append(self.sim_step)
        self.distances_info['Distances'].append(self.current_total_distances_per_bus.copy())

    def _get_total_distances(self):
        for bus in self.active_busses:
            if bus not in self.total_distances_travelled:
                self.total_distances_travelled[bus] = None
            self.total_distances_travelled[bus] = self.distance_travelled[bus] + (
                    self.cycles[bus] * self.tot_network_distance)
        return self.total_distances_travelled

    def _update_rl_performance(self):
        self.rl_performance['Step'].append(self.rl_steps)
        rew = abs(self.last_reward)
        self.rl_performance['Cost Function'].append(rew)

    def _get_headways(self):
        i = 0
        for key in self.active_busses:
            self.headways[key] = -1  # for vehicle_0 at start of simulation because otherwise the headway of the one
            # in front of it would be very large
        for veh in self.busses_distance_travelled:
            if len(self.busses_distance_travelled) == self.total_busses:  # all busses are active in network
                if i != 0:  # not the first bus
                    front_headway_meters = list(self.busses_distance_travelled.values())[i - 1] - \
                                           list(self.busses_distance_travelled.values())[i]
                    front_headway_meters = int(front_headway_meters)
                else:  # bus is the first one but all the vehicles are in network
                    front_headway_meters = (
                                               list(self.busses_distance_travelled.values())[
                                                   len(self.busses_distance_travelled) - 1]) + (
                                                   (300 * 4) - list(self.busses_distance_travelled.values())[i])
                    front_headway_meters = int(front_headway_meters)
                front_headway_time = front_headway_meters / traci.vehicle.getMaxSpeed(veh)  # time = distance/speed
                self.headways[veh] = front_headway_time
            else:  # not all busses are active in network
                if i != 0:
                    front_headway_meters = list(self.busses_distance_travelled.values())[i - 1] - \
                                           list(self.busses_distance_travelled.values())[i]
                    front_headway_meters = int(front_headway_meters)
                    front_headway_time = front_headway_meters / traci.vehicle.getMaxSpeed(veh)  # time = distance/speed
                    self.headways[veh] = int(front_headway_time)
            i += 1

    def _get_upcoming_stop(self, road_idx):
        """
        returns which stop a bus should hold control at depending on where it is in the network
        """
        upcoming_stop = None
        for i in range(len(self.stop_locations)):
            stop_idx = self.stop_locations[list(self.stop_locations.keys())[i]][1]
            if int(stop_idx) > int(road_idx):
                upcoming_stop = list(self.stop_locations.keys())[i]
                break

        if upcoming_stop is None:
            upcoming_stop = list(self.stop_locations.keys())[0]
        return upcoming_stop

    def _get_follower(self, bus):
        dist = self.busses_distance_travelled
        distances_values = sorted(list(dist.values()), reverse=False)
        current_dist = dist[bus]
        follower_idx = distances_values.index(current_dist) - 1
        follower_distance = distances_values[follower_idx]
        follower_bus = list(dist.keys())[list(dist.values()).index(follower_distance)]
        return follower_bus

    def _get_consecutive_busses(self, bus):
        busses = list(self.busses_distance_travelled.keys())
        if busses.index(bus) == len(self.busses_distance_travelled) - 1:  # bus is first
            idx_of_follower = busses.index(bus) - 1
            follower = busses[idx_of_follower]
            leader = busses[0]
        elif busses.index(bus) == 0:  # bus is last
            follower = busses[-1]  # take last element
            idx_of_leader = busses.index(bus) + 1
            leader = busses[idx_of_leader]
        else:
            idx_of_follower = busses.index(bus) - 1
            follower = busses[idx_of_follower]
            idx_of_leader = busses.index(bus) + 1
            leader = busses[idx_of_leader]
        return follower, leader

    def get_passengers(self):
        """
        Get Passengers in all active buses
        """
        for bus in self.active_busses:
            self.passengers[bus] = traci.vehicle.getPersonNumber(bus)

    def _get_veh_capacity(self):
        """returns passenger capacity for each bus"""
        for bus in self.active_busses:
            self.capacities[bus] = traci.vehicle.getPersonCapacity(bus)

    def _get_passengers_waiting(self):
        """returns dict of bus stops with the number of passengers waiting at each one"""
        for bus_stop in self.bus_stops:
            self.passengers_at_each_stop[bus_stop] = traci.busstop.getPersonCount(bus_stop)

    def _get_average_wait_times(self):
        for stop in self.bus_stops:
            people_at_stop = traci.busstop.getPersonIDs(stop)
            no_of_ppl_at_stop = traci.busstop.getPersonCount(stop)
            tot_waiting_time = 0
            self.average_wait_time_at_each_stop[stop] = 0
            if no_of_ppl_at_stop != 0:
                for person in people_at_stop:
                    tot_waiting_time += (traci.person.getWaitingTime(person))
                self.average_wait_time_at_each_stop[stop] = tot_waiting_time / no_of_ppl_at_stop

    def _get_vehs_at_stops(self):
        """vehs_at_each_stop[bus stop] = None if no bus is stopped or bus_name if bus is stopped"""
        for bus_stop in self.bus_stops:
            vehicles = traci.busstop.getVehicleIDs(bus_stop)
            if not vehicles:
                self.vehs_at_each_stop[bus_stop] = None
            else:
                if len(vehicles) == 1:
                    vehicles = vehicles[0]
                self.vehs_at_each_stop[bus_stop] = vehicles

    def _get_vehs_who_are_stopped(self):
        """
        vehs_who_are_stopped[bus][0] = is stopped at current time step?: bool
        vehs_who_are_stopped[bus][1] = was stopped at last time step?: bool
        """
        for active_bus in self.active_busses:
            if active_bus not in list(self.vehs_who_are_stopped.keys()):
                self.vehs_who_are_stopped[active_bus] = [False, False]

        for bus in self.active_busses:
            last_time_step = self.vehs_who_are_stopped[bus][0]
            self.vehs_who_are_stopped[bus][0] = traci.vehicle.isAtBusStop(bus)
            self.vehs_who_are_stopped[bus][1] = last_time_step

    def get_high_low_pass_p_stop(self):
        """
        Gets range for bus stops (used for initialisation of observation space)
        """
        pass_high = []
        pass_low = []
        for stop in self.bus_stops:
            pass_high.append(float('inf'))
            pass_low.append(float(0))
        return pass_high, pass_low

    def get_high_low_ohe_bus_stop(self):
        """
        Gets range for one hot encoded current stop (used for initialisation of observation space)
        """
        bus_stop_high = []
        bus_stop_low = []
        for stop in self.bus_stops:
            bus_stop_high.append(float(1))
            bus_stop_low.append(float(0))
        return bus_stop_high, bus_stop_low
