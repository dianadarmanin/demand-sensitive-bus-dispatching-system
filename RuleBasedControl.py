import os, sys
import time

import traci
import sumolib


def colored(r, g, b, text):
    return "\033[38;2;{};{};{}m{} \033[38;2;255;255;255m".format(r, g, b, text)


# returns the distance a vehicle has travelled in one cycle: from gneE0 to gneE11
def get_distance_on_net(veh_id):
    lane_pos = traci.vehicle.getLanePosition(veh_id)
    road_id = traci.vehicle.getRoadID(veh_id)
    if "E" in road_id:  # its an edge because it contains an E
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


# returns a dict of busses with the front headway for each bus
def get_headways(active_busses, headways, busses_distance_travelled, TOTAL_BUSSES):
    i = 0
    for key in active_busses:
        headways[key] = -1  # for vehicle_0 at start of simulation because otherwise the headway of the one
        # in front of it would be very large
    for veh in busses_distance_travelled:
        if len(busses_distance_travelled) == TOTAL_BUSSES:  # all busses are active in network
            if i != 0:  # not the first bus
                front_headway_meters = list(busses_distance_travelled.values())[i - 1] - \
                                       list(busses_distance_travelled.values())[i]
                front_headway_meters = int(front_headway_meters)
            else:  # bus is the first one but all the vehicles are in network
                front_headway_meters = (
                                           list(busses_distance_travelled.values())[
                                               len(busses_distance_travelled) - 1]) + (
                                               (300 * 4) - list(busses_distance_travelled.values())[i])
                front_headway_meters = int(front_headway_meters)
            front_headway_time = front_headway_meters / traci.vehicle.getMaxSpeed(veh)  # time = distance/speed
            headways[veh] = front_headway_time
        else:  # not all busses are active in network
            if i != 0:
                front_headway_meters = list(busses_distance_travelled.values())[i - 1] - \
                                       list(busses_distance_travelled.values())[i]
                front_headway_meters = int(front_headway_meters)
                front_headway_time = front_headway_meters / traci.vehicle.getMaxSpeed(veh)  # time = distance/speed
                headways[veh] = int(front_headway_time)
        i += 1

    return headways


# returns color coded text according to sumo-gui output
def color_bus(bus):
    if bus == 'vehicle_0':
        colored_txt = colored(0, 255, 0, bus)
    elif bus == 'vehicle_1':
        colored_txt = colored(51, 255, 255, bus)
    elif bus == 'vehicle_2':
        colored_txt = colored(255, 0, 0, bus)
    elif bus == 'vehicle_3':
        colored_txt = colored(255, 255, 0, bus)
    return colored_txt


# returns dict of busses with the number of passengers occupying each one
def get_passengers():
    passengers = dict()
    active_busses = traci.vehicle.getIDList()
    for bus in active_busses:
        passengers[bus] = traci.vehicle.getPersonNumber(bus)
    return passengers


# returns dict of bus stops with the number of passengers waiting at each one
def get_passengers_waiting(BUS_STOPS):
    passengers_waiting = dict()
    for bus_stop in BUS_STOPS:
        passengers_waiting[bus_stop] = traci.busstop.getPersonCount(bus_stop)
    return passengers_waiting


# returns two dicts:    1. Dict of bus stops indicating vehicle stopped at it, None otherwise
#                       2. Dict of busses and boolean indicating if each bus is at bus stop or not
def get_vehs_at_stops(BUS_STOPS):
    vehs_at_each_stop = dict()
    for bus_stop in BUS_STOPS:
        vehicles = traci.busstop.getVehicleIDs(bus_stop)
        if not vehicles:
            vehs_at_each_stop[bus_stop] = None
        else:
            if len(vehicles) == 1:
                vehicles = vehicles[0]
            vehs_at_each_stop[bus_stop] = vehicles

    return vehs_at_each_stop


def get_vehs_who_are_stopped(active_busses, vehs_who_are_stopped):
    for active_bus in active_busses:
        if active_bus not in list(vehs_who_are_stopped.keys()):
            vehs_who_are_stopped[active_bus] = [False, False]

    for bus in active_busses:
        last_time_step = vehs_who_are_stopped[bus][0]
        vehs_who_are_stopped[bus][0] = traci.vehicle.isAtBusStop(bus)
        vehs_who_are_stopped[bus][1] = last_time_step
    return vehs_who_are_stopped


# returns passenger capacity for each bus
def get_veh_capacity(active_busses):
    capacities = dict()
    for bus in active_busses:
        capacities[bus] = traci.vehicle.getPersonCapacity(bus)
    return capacities


# returns stop at which vehicle is stopped at
def get_current_stop(veh_id, vehs_at_each_stop):
    for stop in vehs_at_each_stop:
        if veh_id == vehs_at_each_stop[stop]:
            return stop


# # updates the busy_busses dict: if bus has just stopped, log timestamp
# def update_busy(curr_time, vehs_who_are_stopped):
#     for bus in busy_busses:
#         if busy_busses[bus] is None and vehs_who_are_stopped[bus] is True:
#             busy_busses[bus] = curr_time
#         elif vehs_who_are_stopped[bus] is False:
#             busy_busses[bus] = None


# returns the waiting time for each bus. A vehicle that is stopping intentionally with a <stop> does not accumulate waiting time.
def get_waiting_time(active_busses):
    waiting_time = dict()
    for bus in active_busses:
        waiting_time[bus] = traci.vehicle.getWaitingTime(bus)
    return waiting_time


# lane_ids are in the format of gneE0_1, gneE0_0
# road_ids are in the format of gneE0, gneE1...
def get_road_from_lane(lane_id):
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


def get_bus_stop_locations():
    # returns a dict of bus stops whose values are tuples with (road id, road index): example (gneE4, 4)
    bus_stop_locations = dict()
    for bus_stop in traci.busstop.getIDList():
        lane_id = traci.busstop.getLaneID(bus_stop)
        bus_stop_locations[bus_stop] = get_road_from_lane(lane_id)
    return bus_stop_locations


def get_upcoming_stop(road_idx, stop_locations):
    # returns which stop a bus should hold control at depending on where it is in the network
    upcoming_stop = None
    for i in range(len(stop_locations)):
        stop_idx = stop_locations[list(stop_locations.keys())[i]][1]
        if int(stop_idx) > int(road_idx):
            upcoming_stop = list(stop_locations.keys())[i]
            break
        else:
            continue
    if upcoming_stop is None:
        upcoming_stop = list(stop_locations.keys())[0]
    return upcoming_stop


# returns a dict of busses with the list of passengers in each bus
def get_persons_in_vehs(active_busses):
    persons_in_vehicles = dict()
    for bus in active_busses:
        persons_in_vehicles[bus] = traci.vehicle.getPersonIDList(bus)
    return persons_in_vehicles


#
# # returns the number of passengers alighting the vehicle at current timestep
# def get_no_of_alighting(bus, persons_in_vehs):
#     alighting[bus] = 0
#     current_edge = traci.vehicle.getRoadID(bus)
#     current_stop = get_current_stop(bus)
#     for person in persons_in_vehs[bus]:  # for every person within that bus
#         if traci.person.getEdges(person)[-1] == current_edge:  # if person's last edge is same as bus' current edge
#             alighting[bus] += 1  # increment number of persons alighting
#     return alighting, current_stop
#

# def update_cycles(active_busses, cycle_history, cycles, first_edge, last_edge):
#     if 'vehicle_0' not in active_busses:
#         pass
#     else:
#         road_id, road_idx = get_road_from_lane(traci.vehicle.getLaneID('vehicle_0'))
#
#         if cycle_history[1] != road_idx:
#             cycle_history[0] = int(cycle_history[1])
#             cycle_history[1] = int(road_idx)
#
#         if cycle_history[0] == last_edge and cycle_history[1] == first_edge:
#             cycles += 1
#     return cycle_history, cycles
def update_cycles(active_busses, cycle_history, cycles, first_edge, last_edge):
    for bus in active_busses:
        # if 'vehicle_0' not in active_busses:
        #     pass
        # else:
        road_id, road_idx = get_road_from_lane(traci.vehicle.getLaneID(bus))

        if cycle_history[bus][1] != road_idx:
            cycle_history[bus][0] = int(cycle_history[bus][1])
            cycle_history[bus][1] = int(road_idx)

        if cycle_history[bus][0] == last_edge and cycle_history[bus][1] == first_edge:
            cycles[bus] += 1
            if bus == 'vehicle_0':
                print("Cycles completed by vehicle 0: ", cycles[bus])
    return cycle_history, cycles

def print_time(time):
    print(colored(0, 255, 0, "=======\t" + str(time) + "\t======="))


def get_follower(busses_distance_travelled, bus):
    dist = busses_distance_travelled
    distances_values = sorted(list(dist.values()), reverse=False)
    current_dist = dist[bus]
    follower_idx = distances_values.index(current_dist) - 1
    follower_distance = distances_values[follower_idx]
    follower_bus = list(dist.keys())[list(dist.values()).index(follower_distance)]
    return follower_bus


def headway_variability_reward(headways):
    mean = sum(list(headways.values())) / len(headways)
    numerator = 0
    for headway in headways:
        numerator += ((headways[headway] - mean) ** 2)
    variance = numerator / len(headways)
    reward = - variance
    return reward


def waiting_time_variability_reward(bus_stops):
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


def update_performance(performance, time, active_busses, headways, bus_stops, total_dist_travelled, distance_travelled,
                       cycles,
                       tot_network_distance):
    performance['Time'].append(time)
    performance['Headways'].append(headways)
    performance['Headway Variance'].append(headway_variability_reward(headways))
    performance['Average Wait Time Variance'].append(waiting_time_variability_reward(bus_stops))
    performance['Total Distances Travelled'].append(
        get_total_distances(active_busses, total_dist_travelled, distance_travelled, cycles, tot_network_distance))
    return performance


def get_total_distances(active_busses, total_dist_travelled, distance_travelled, cycles, tot_network_distance):
    for bus in active_busses:
        if bus not in total_dist_travelled:
            total_dist_travelled[bus] = None
        total_dist_travelled[bus] = distance_travelled[bus] + (cycles[bus] * tot_network_distance)
    return total_dist_travelled
