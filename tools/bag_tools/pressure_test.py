#!/usr/bin/python
import sys
import rosbag
import numpy as np
import argparse as argp
import matplotlib.pyplot as plt
from collections import defaultdict


def make_pair(x):
    return x, x


def process_data(data_dict):
    fig, ax = plt.subplots()
    for key in data_dict:
        time = data_dict[key]['time']
        data = data_dict[key]['data']
        # plot all data
        line = ax.plot(time, data)
        color = plt.getp(line[0], 'color')
        average = np.mean(data)
        std = np.std(data)
        # plot average and variance
        time_beg_to_end = (time[0], time[-1])
        ax.plot(time_beg_to_end, make_pair(average), '--', color=color,
                linewidth=4)
        ax.plot(time_beg_to_end, make_pair(average + std), '-.', color=color,
                linewidth=2)
        ax.plot(time_beg_to_end, make_pair(average - std), '-.', color=color,
                linewidth=2)
        print '{0} average: {1}, std: {2}'.format(key, average, std)

    plt.grid()
    plt.show()


def process_bag(bagfile, topics):
    pressure_data = defaultdict(lambda: defaultdict(list))
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages():
        if topic in topics:
            pressure_data[topic]['data'].append(msg.fluid_pressure)
            pressure_data[topic]['time'].append(msg.header.stamp.to_sec())
    bag.close()
    for topic in topics:
        print '{0} contains {1} pressure messages.'.format(topic, len(
            pressure_data[topic]))
    # process pressure data for average and variance
    process_data(pressure_data)


def main():
    parser = argp.ArgumentParser(description='Process imu pressure data')
    parser.add_argument('bagfile')
    parser.add_argument('-t', '--topics', nargs='*', default='/imu/pressure')
    parsed_args = parser.parse_args()
    process_bag(parsed_args.bagfile, parsed_args.topics)


if __name__ == '__main__':
    sys.exit(main())
