import matplotlib.pyplot as plt


#  analyse performance
def pre_process_distances(distances):
    """
    :param distances:
    :type distances:
    :return:
    :rtype:
    """
    distances_per_bus = dict()
    all_busses = set()
    log_counter = 0
    for log in distances:
        busses_in_log = list(log.keys())
        for bus in busses_in_log:
            if bus not in list(distances_per_bus.keys()):
                distances_per_bus[bus] = []
            if bus not in all_busses:
                zeroes = [0 for i in range(log_counter)]
                distances_per_bus[bus].extend(zeroes)
            distances_per_bus[bus].append(log[bus])
            if bus not in all_busses:
                all_busses.add(bus)
        log_counter += 1

    return distances_per_bus


def plot_dist_time_graph(performance, control_strategy):
    """
    :param performance:
    :type performance:
    :param control_strategy:
    :type control_strategy:
    """
    distances_per_bus = pre_process_distances(performance['Distances'])
    time = [t/60 for t in performance['Time']]
    plt.figure(figsize=(20, 20))

    for k, v in distances_per_bus.items():
        new_time = time[0:len(v)]
        plt.plot(new_time, v, '.-', label=k)
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))  # To draw legend
    plt.title("Time against Distance Graph depicting buses' trajectories for " + control_strategy)
    plt.xlabel("Time (m)")
    plt.ylabel("Distance (m)")
    plt.savefig("Results" + '\\' + control_strategy + '_dist_time.png', bbox_inches='tight')


def plot_dist_time_graph_spec(distances_info, control_strategy):
    """
    :param distances_info:
    :type distances_info:
    :param control_strategy:
    :type control_strategy:
    """
    distances_per_bus = pre_process_distances(distances_info['Distances'])
    time = [t/60 for t in distances_info['Time']]
    plt.figure(figsize=(20, 20))

    for k, v in distances_per_bus.items():
        new_time = time[0:len(v)]
        plt.plot(new_time, v, '.-', label=k)

    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))  # To draw legend
    plt.title("Time against Distance Graph depicting buses' trajectories for " + control_strategy)
    plt.xlabel("Time (m)")
    plt.ylabel("Distance (m)")
    plt.savefig("Results" + '\\' + control_strategy + '_dist_time.png', bbox_inches='tight')


def plot_reward1_time_graph(performance, control_strategy):
    """
    :param performance:
    :type performance:
    :param control_strategy:
    :type control_strategy:
    """
    time = [t/60 for t in performance['Time']]
    y1 = [abs(r) for r in performance['Headway Variance']]
    plt.figure(figsize=(10, 6.75))
    plt.plot(time, y1, label="R1")
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))  # To draw legend
    plt.title("Time against Absolute Reward 1 Graph for " + control_strategy)
    plt.xlabel("Time (m)")
    plt.ylabel("Reward")
    plt.savefig("Results" + '\\' + control_strategy + '_reward1_time.png', bbox_inches='tight')


def plot_reward2_time_graph(performance, control_strategy):
    """
    :param performance:
    :type performance:
    :param control_strategy:
    :type control_strategy:
    """
    time = [t/60 for t in performance['Time']]
    y2 = [abs(r) for r in performance['Average Wait Time Variance']]
    plt.figure(figsize=(10, 6.75))
    plt.plot(time, y2, label="R2")
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))  # To draw legend
    plt.title("Time against Absolute Reward 2 Graph for " + control_strategy)
    plt.xlabel("Time (m)")
    plt.ylabel("Reward")
    plt.savefig("Results" + '\\' + control_strategy + '_reward2_time.png', bbox_inches='tight')


def plot_cost_steps_graph(rl_performance,control_strategy):
    """
    :param rl_performance:
    :type rl_performance:
    """
    steps = rl_performance['Step']
    reward = [abs(r) for r in rl_performance['Cost Function']]
    plt.figure(figsize=(10, 10))

    plt.plot(steps, reward)
    plt.title("Reward acquired during Training of DQN")
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.savefig("Results" + '\\' +control_strategy+ 'cost_step_dqn.png', bbox_inches='tight')


def progress_print(curr_time, active_busses, start_time, max_time=100000):
    perc = (curr_time-start_time)/ (max_time-start_time)
    prog = perc * 10
    prog_bar = ""
    for i in range(1, 11):
        if i <= prog:
            prog_bar += "⚫"
        else:
            prog_bar += "⚪"
    print("Step ", curr_time, "[", prog_bar, "]", perc * 100, "% with ", str(len(active_busses))," active busses")
