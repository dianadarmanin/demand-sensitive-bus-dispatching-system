import os, sys
import time
import traci
import sumolib
import matplotlib.pyplot as plt


def _headway_variability_reward(headways):
    mean = sum(list(headways.values())) / len(headways)
    numerator = 0
    for headway in headways:
        numerator += ((headways[headway] - mean) ** 2)
    variance = numerator / len(headways)
    reward = - variance
    return reward


def _waiting_time_variability_reward(bus_stops):
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


# returns the distance a vehicle has travelled in one cycle: from gneE0 to gneE11
def get_distance_on_net(veh_id):
    """

    :param veh_id:
    :type veh_id:
    :return:
    :rtype:
    """
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


# lane_ids are in the format of gneE0_1, gneE0_0
# road_ids are in the format of gneE0, gneE1...
def get_road_from_lane(lane_id):
    """

    :param lane_id:
    :type lane_id:
    :return:
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


# returns a dict of busses with the front headway for each bus
def get_headways(active_busses, busses_distance_travelled, headways, TOTAL_BUSSES):
    """

    :param active_busses:
    :type active_busses:
    :param busses_distance_travelled:
    :type busses_distance_travelled:
    :param headways:
    :type headways:
    :param TOTAL_BUSSES:
    :type TOTAL_BUSSES:
    :return:
    :rtype:
    """
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


def update_cycles(active_busses, cycle_history, cycles, first_edge, last_edge):
    """

    :param active_busses:
    :type active_busses:
    :param cycle_history:
    :type cycle_history:
    :param cycles:
    :type cycles:
    :param first_edge:
    :type first_edge:
    :param last_edge:
    :type last_edge:
    :return:
    :rtype:
    """
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


def update_performance(performance, time, active_busses, headways, bus_stops, total_dist_travelled, distance_travelled,
                       cycles,
                       tot_network_distance):
    """

    :param performance:
    :type performance:
    :param time:
    :type time:
    :param active_busses:
    :type active_busses:
    :param headways:
    :type headways:
    :param bus_stops:
    :type bus_stops:
    :param total_dist_travelled:
    :type total_dist_travelled:
    :param distance_travelled:
    :type distance_travelled:
    :param cycles:
    :type cycles:
    :param tot_network_distance:
    :type tot_network_distance:
    :return:
    :rtype:
    """
    tot_dist = get_total_distances(active_busses, total_dist_travelled, distance_travelled, cycles, tot_network_distance)
    performance['Time'].append(time)
    performance['Headways'].append(headways)
    performance['Headway Variance'].append(_headway_variability_reward(headways))
    performance['Average Wait Time Variance'].append(_waiting_time_variability_reward(bus_stops))
    performance['Total Distances Travelled'].append(tot_dist)
    return performance, tot_dist


def get_total_distances(active_busses, total_dist_travelled, distance_travelled, cycles, tot_network_distance):
    """

    :param active_busses:
    :type active_busses:
    :param total_dist_travelled:
    :type total_dist_travelled:
    :param distance_travelled:
    :type distance_travelled:
    :param cycles:
    :type cycles:
    :param tot_network_distance:
    :type tot_network_distance:
    :return:
    :rtype:
    """
    for bus in active_busses:
        if bus not in total_dist_travelled:
            total_dist_travelled[bus] = None
        total_dist_travelled[bus] = distance_travelled[bus] + (cycles[bus] * tot_network_distance)
    return total_dist_travelled
