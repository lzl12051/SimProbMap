import logging

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st

from ProbMap import ProbMap, ProbMapData
from Robot import Robot


class Sensor:
    def __init__(self, tracker, coverage_radius=150) -> None:
        self.type = 'Cam'
        # host tracker
        self.tracker = tracker
        self.coverage_radius = coverage_radius

    def get_detection(self):
        all_targets = self.tracker.simulator.targets
        detections = []
        for target in all_targets:
            if np.linalg.norm(target.position - self.tracker.position) < self.coverage_radius:
                detection = (target.position-self.tracker.position)
                # logging.debug(
                #     f"Ture {self.tracker.name}{self.tracker.id} detection:\t" + str(detection))
                # make up some noise and calculate the confidence of the detection
                std_dev = 1
                noise = np.random.normal(loc=0, scale=std_dev, size=2)
                confidence = sum(st.norm.pdf(
                    noise, loc=0, scale=std_dev)*2)/2*std_dev
                if confidence <= 0.55:
                    confidence = 0.55
                detection = detection + noise
                distance = np.linalg.norm(detection)
                if distance > self.coverage_radius:
                    detection = detection * (self.coverage_radius/distance)
                detection = np.append(detection, confidence)
                detections.append(detection)
                logging.debug(
                    f"Noisy {self.tracker.log_head} Detection: {detection}")
        return detections


class Tracker(Robot):
    def __init__(self, simulator, name: str, id: int, position: np.array, coverage_radius) -> None:
        super().__init__(simulator, name, id, position)
        self.sensor = Sensor(self, coverage_radius)
        self.neighbor = set()

        self.area_width = 2000  # meter
        self.area_height = 2000  # meter
        self.resolution = 0.5  # meter
        self.prob_map = ProbMap(self.area_width, self.area_height, self.resolution,
                                center_x=0.0, center_y=0.0, init_val=0.05,
                                false_alarm_prob=0.01)

        self.observations = dict()  # type: dict[tuple]
        self.shareable_v = ProbMapData()
        self.shareable_Q = ProbMapData()
        self.neighbors_v = dict()
        self.neighbors_Q = dict()

    def build_shareable_info(self, shareable_info, info_type):
        """Generate shareable information from local

        Args:
            shareable_info (dict): Stores all local infomation. Format: {(x, y) : value}
        """
        local_meas_info = ProbMapData()
        local_meas_info.tracker_id = self.id
        local_meas_info.type = info_type
        for k, v in shareable_info.items():
            local_meas_info.grid_ind += k
            local_meas_info.values.append(v)
        self.shareable_v = local_meas_info

    def get_info_from_neighbors(self, req_type):
        neighbors_info = dict()
        # Send requests and get responses from all neighbors' services
        # Collect info from neighbors
        if req_type == 'v':
            for e in self.neighbor:
                self.neighbors_v[e] = self.simulator.trackers[e].shareable_v

            for _id, res in self.neighbors_v.items():
                for i in range(len(res.values)):
                    cell_ind = tuple([res.grid_ind[i*2], res.grid_ind[i*2+1]])
                    # sum up all neighbors' measurement values
                    value = res.values[i]
                    try:
                        neighbors_info[cell_ind] += value
                    except KeyError:
                        neighbors_info[cell_ind] = value
        elif req_type == 'Q':
            for e in self.neighbor:
                self.neighbors_v[e] = self.simulator.trackers[e].shareable_Q
            for _id, res in self.neighbors_Q.items():
                for i in range(len(res.values)):
                    cell_ind = tuple([res.grid_ind[i*2], res.grid_ind[i*2+1]])
                    # sum up all neighbors' values and counting, need to calculate average value
                    value = res.values[i]
                    try:
                        neighbors_info[cell_ind][0] += value
                        neighbors_info[cell_ind][1] += 1.
                    except KeyError:
                        neighbors_info[cell_ind] = [value, 1.]
        return neighbors_info

    def sensing(self):
        detections = self.sensor.get_detection()
        output_detection = dict()
        id_counter = 0
        for det in detections:
            transformed_detection = det+np.append(self.position, 0)
            output_detection[id_counter] = transformed_detection
            id_counter += 1
        self.observations = output_detection

    def job(self):
        self.sensing()
        logging.debug(f"{self.log_head} OBSERVATION: {self.observations}")
        shareable_v = self.prob_map.generate_shareable_v(
            self.observations)

        # build shareable_v and publish it
        self.build_shareable_info(shareable_v, 'v')
        # logging.debug(f"{self.name}_{self.id}: {self.shareable_v.grid_ind}")

        # get all neighbors' detections
        neighbors_meas = self.get_info_from_neighbors('v')
        # logging.debug("{}{} got neighbor {} info: {}".format(
        #     self.name, self.id, self.neighbor, neighbors_meas))

        # # Update the local map by all detections (local and neighbors')
        self.prob_map.map_update(shareable_v, neighbors_meas,
                                 len(self.simulator.trackers), len(self.neighbor))

        # Convert prob map to a shareable information and publish it
        self.build_shareable_info(
            self.prob_map.non_empty_cell, 'Q')

        # Collect neighbors' map (Q) for consensus
        neighbors_map = self.get_info_from_neighbors('Q')
        # # rospy.loginfo("{} got neighbors' map: {}".format(
        # #     self.name, neighbors_map))

        # Make consensus, merge neighbors' map
        self.prob_map.consensus(neighbors_map)

        self.target_estimates = self.prob_map.get_target_est(
            0.6, normalization=True)
        logging.debug(
            f"{self.name}_{self.id} ProbMap: {self.prob_map.prob_map}")
        # print(target_estimates)
