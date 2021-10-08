import matplotlib.pyplot as plt
import numpy as np
from target import Target
from tracker import Tracker


class Simsim:
    def __init__(self) -> None:
        self.trackers = []
        self.edges = []
        self.targets = []
        self.map_size = [1000, 1000]
        self.rate = 30

        plt.figure(figsize=(8, 8))
        plt.ion()
        plt.axis('equal')

    def add_tracker(self, name, position):
        tracker = Tracker(name, len(self.trackers), position)
        self.trackers.append(tracker)

    def add_edges(self, edges):
        self.edges.extend(edges)
        for e in edges:
            self.trackers[e[0]].neighbor.append(e[1])
            self.trackers[e[1]].neighbor.append(e[0])

    def add_target(self, name, position):
        target = Target(name, len(self.targets), position)
        self.targets.append(target)

    def run(self):
        while 1:
            plt.cla()
            plt.xlim(0, self.map_size[0])
            plt.ylim(0, self.map_size[1])

            for t in self.trackers:
                plt.scatter(t.position[0], t.position[1],
                            marker='s', s=20, c='b')
                plt.annotate(t.id, (t.position[0], t.position[1]+10))
                # draw the camera coverage
                circle = plt.Circle(
                    (t.position), t.sensor.coverage_radius, fill=False, color='grey', alpha=0.3)
                plt.gcf().gca().add_artist(circle)
                # draw the detections
                dets = t.sensor.get_detection(self.targets)
                for det in dets:
                    plt.gcf().gca().scatter(
                        det[0], det[1], marker='^', s=8)  # , alpha=det[2]
                t.run(self.rate)

            for e in self.edges:
                t0 = self.trackers[e[0]]
                t1 = self.trackers[e[1]]
                plt.plot([t0.position[0], t1.position[0]],
                         [t0.position[1], t1.position[1]],
                         linewidth=1, color='g', alpha=0.5)

            plt.draw()
            plt.pause(1/self.rate)


s = Simsim()
s.add_tracker('t', np.array([120.0, 180.0]))
s.trackers[0].speed = np.array([10,10])
s.add_tracker('t', np.array([220, 380]))
s.trackers[1].speed = np.array([5,10])
s.add_tracker('t', np.array([500, 380]))
s.trackers[2].speed = np.array([-22,10])
s.add_target('tgt', np.array([200, 300]))
s.add_target('tgt', np.array([100, 300]))
s.add_target('tgt', np.array([500, 300]))
s.add_edges([[0, 1], [0, 2]])
s.run()
