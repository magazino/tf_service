#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import curses
import datetime
import os
import time

import pandas as pd
import psutil
import rosgraph
import rosnode

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy


def pid_from_node_name(node_name):
    caller_id = "monitor"
    master = rosgraph.Master(caller_id, master_uri=os.getenv("ROS_MASTER_URI"))
    node_api = rosnode.get_api_uri(master, node_name)
    if node_api is None:
        raise NameError("unknown node name '{}'".format(node_name))
    node = ServerProxy(node_api)
    pid = rosnode._succeed(node.getPid(caller_id))
    return pid


class Processes(object):
    def __init__(self, pids):
        self.processes = [psutil.Process(pid) for pid in pids]
        for process in self.processes:
            process.cpu_percent()

    @property
    def size(self):
        return len(self.processes)

    def get(self, attribute):
        if not callable(getattr(psutil.Process, attribute)):
            return [getattr(process, attribute) for process in self.processes]
        return [getattr(process, attribute)() for process in self.processes]


class RosProcessMonitor(object):
    def __init__(self, nodes=None):
        self.nodes = None
        self.processes = None
        self.refresh_process_list(nodes)

    def refresh_process_list(self, nodes=None):
        if nodes:
            new_nodes = list(set(nodes) & set(rosnode.get_node_names()))
        else:
            new_nodes = rosnode.get_node_names()
        if new_nodes == self.nodes:
            return
        self.nodes = new_nodes
        pids = []
        for node in self.nodes:
            try:
                pids.append(pid_from_node_name(node))
            except:
                pass
        self.processes = Processes(pids)

    def dataframe(self, attributes):
        if self.processes.size == 0:
            return pd.DataFrame()
        data = {attr: self.processes.get(attr) for attr in attributes}
        now = datetime.datetime.now()
        index = pd.MultiIndex.from_product([[now], self.nodes], names=("time",
                                                                       "node"))
        return pd.DataFrame(index=index, data=data)


if __name__ == "__main__":
    import argparse
    choices = ("username", "pid", "parent", "cpu_percent", "cpu_num",
               "num_threads", "memory_percent", "name")
    parser = argparse.ArgumentParser()
    parser.add_argument("--nodes", nargs='*', default=None)
    parser.add_argument("--attributes", default=[
        "username", "pid", "cpu_percent", "num_threads", "memory_percent"
    ], nargs='+', choices=choices)
    parser.add_argument("-s", "--sort-by", default="cpu_percent",
                        choices=choices)
    parser.add_argument("--sort-ascending", action="store_true")
    parser.add_argument("-r", "--refresh-rate", default=1, type=float)
    parser.add_argument("-p", "--plot", action="store_true")
    parser.add_argument("--plot-attribute", default="cpu_percent",
                        choices=choices)
    args = parser.parse_args()

    monitor = RosProcessMonitor(args.nodes)

    pd.options.display.float_format = "{:,.2f}".format
    fig, ax = None, None
    xs, ys = [], []

    stdscr = curses.initscr()
    stdscr.idlok(1)
    stdscr.scrollok(True)

    aggregate_df = pd.DataFrame()
    try:
        while True:
            time.sleep(1. / args.refresh_rate)
            
            monitor.refresh_process_list(args.nodes)
            try:
                df = monitor.dataframe(args.attributes)
            except psutil.NoSuchProcess:
                continue
            if df.empty:
                continue
            if args.plot:
                aggregate_df = pd.concat((aggregate_df, df))
            df.index = df.index.droplevel()
            df = df.sort_values(args.sort_by, ascending=args.sort_ascending)
            max_x, max_y = stdscr.getmaxyx()
            if curses.is_term_resized(max_x, max_y):
                curses.resizeterm(max_x, max_y)
                stdscr.clear()
            stdscr.refresh()
            stdscr.addstr(0, 0, df.to_string(max_cols=max_x))

    except KeyboardInterrupt:
        pass
    finally:
        curses.endwin()

    if args.plot and not aggregate_df.empty:
        import matplotlib
        matplotlib.use("Qt5Agg")
        import matplotlib.pyplot as plt
        import seaborn as sns
        sns.set()
        sns.set_palette("tab20", n_colors=12)
        plot_df = aggregate_df.unstack()[args.plot_attribute]
        ax = plot_df.plot(kind="line")
        ax.set_ylabel(args.plot_attribute)
        plt.show()
