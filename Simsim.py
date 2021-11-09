import logging
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator

from target import Target
from tracker import Tracker

import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)


class Simsim:
    class Topics:
        def __init__(self) -> None:
            self.topics = dict()

        def add_topic(self, topic_name):
            self.topics[topic_name] = deque(maxlen=20)

        def del_topic(self, topic_name):
            try:
                del self.topics[topic_name]
            except:
                pass

        def publish(self, topic_name, msg):
            self.topics[topic_name].append(msg)

        def subs(self, topic_name):
            return self.topics[topic_name]

    def __init__(self) -> None:
        self.trackers = []
        self.edges = []
        self.targets = []
        self.map_size = [1000, 1000]
        self.rate = 30
        self.topics = self.Topics()

        plt.ion()

        # self.fig, (self.plt_sim, self.plt_pm) = plt.subplots(
        #     1, 2, figsize=(10, 5), dpi=160)
        fig = plt.figure(figsize=(20, 8))
        self.plt_sim = fig.add_subplot(121)
        # plt.axis('equal')
        self.plt_pm = fig.add_subplot(122, projection='3d')
        # plt.axis('equal')
        self.plt_sim.axis('equal')

    def add_tracker(self, name, position, sensor_rad):
        tracker = Tracker(self, name, len(self.trackers),
                          position, sensor_rad)
        self.trackers.append(tracker)

    def add_edges(self, edges):
        self.edges.extend(edges)
        for e in edges:
            self.trackers[e[0]].neighbor.add(e[1])
            self.trackers[e[1]].neighbor.add(e[0])

    def add_target(self, name, position):
        target = Target(self, name, len(self.targets), position)
        self.targets.append(target)

    def _update_all(self):
        """Simulate once, update all trackers and targets
        """
        for tracker in self.trackers:
            tracker.job()
        for target in self.targets:
            target.job()

    def run(self, log_lvl=logging.WARN, ground_truth=False):
        logging.basicConfig(format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s',
                            datefmt='%m/%d/%Y %H:%M:%S', level=log_lvl)
        while 1:
            self._update_all()
            self.plt_sim.cla()
            self.plt_pm.cla()
            self.plt_sim.set_xlim(0, self.map_size[0])
            self.plt_sim.set_ylim(0, self.map_size[1])
            self.plt_pm.set_xlim(1000, 2000)
            self.plt_pm.set_ylim(1000, 2000)

            if ground_truth:
                for target in self.targets:
                    self.plt_sim.scatter(
                        target.position[0], target.position[1], marker='x', s=2, color='r')
            for e in self.edges:
                t0 = self.trackers[e[0]]
                t1 = self.trackers[e[1]]
                self.plt_sim.plot([t0.position[0], t1.position[0]],
                                  [t0.position[1], t1.position[1]],
                                  linewidth=1, color='g', alpha=0.5)
            Z = np.zeros([2000, 2000])
            for t in self.trackers:
                # draw the trackers
                self.plt_sim.scatter(t.position[0], t.position[1],
                                     marker='s', s=20, c='b')
                self.plt_sim.annotate(t.id, (t.position[0], t.position[1]+10))
                # draw the camera coverage
                circle = plt.Circle(
                    (t.position), t.sensor.coverage_radius, fill=False, color='grey', alpha=0.3)
                self.plt_sim.add_patch(circle)
                for est in t.target_estimates:
                    det_pos = est[0:2]
                    # det_abs_pos = det_pos+t.position
                    self.plt_sim.scatter(det_pos[0], det_pos[1],
                                         marker='^', s=2)
                # t = self.trackers[2]
                for ind in t.prob_map.prob_map:
                    Z[ind] = t.prob_map.prob_map[ind]
                    for dx in [-2, -1, 0, 1, 2]:
                        for dy in [-2, -1, 0, 1, 2]:
                            if Z[ind[0]+dx, ind[1]+dy] <= t.prob_map.prob_map[ind]:
                                Z[ind[0]+dx, ind[1] +
                                    dy] = t.prob_map.prob_map[ind]
            # print(Z)
            X = np.arange(1000, 2000, 1)
            Y = np.arange(1000, 2000, 1)
            X, Y = np.meshgrid(X, Y)
            self.plt_pm.set_zlim(0, 1.01)
            # print(Z)
            self.plt_pm.plot_surface(
                X, Y, Z[1000:, 1000:], cmap='RdBu_r', rcount=150, ccount=150, antialiased=True)
            self.plt_pm.view_init(elev=35., azim=0)
            plt.gca().invert_yaxis()
            # self.plt_pm.plot_surface(X, Y, Z, cmap=cm.bwr, linewidth=5, antialiased=True)
            self.plt_pm.zaxis.set_major_locator(LinearLocator(10))
            plt.pause(1/self.rate)
