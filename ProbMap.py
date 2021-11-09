#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import logging

"""
Tracking implementation for the perimeter monitoring problem

Implementations
--------------- 
1. ProbMap := Probability map for estimating targets position

References
----------
[1] Hu, J.; Xie, L.; Lum, K.Y.; Xu, J. Multiagent Information Fusion and 
    Cooperative Control in Target Search. IEEE Trans. Control Syst. Technol.
    2013, 21, 1223–1235.

"""


class ProbMap:

    def __init__(self, width_meter, height_meter, resolution,
                 center_x, center_y, init_val=0.01, false_alarm_prob=0.05):
        """Generate a probability map

        Args:
            width_meter (int): width of the area [m]
            height_meter (int): height of the area [m]
            resolution (float): grid resolution [m]
            center_x (float): center x position  [m]
            center_y (float): center y position  [m]
            init_val (float, optional): Initial value for all cells. Defaults to 0.01.
            false_alarm_prob (float, optional): False alarm probability of the detector. Defaults to 0.05.
        """
        # TODO make this grid map unlimited, deprecate the width and height params
        # number of cells for width
        self.width = int(np.ceil(width_meter / resolution))
        # number of cells for height
        self.height = int(np.ceil(height_meter / resolution))
        self.resolution = resolution
        self.center_x = center_x
        self.center_y = center_y
        self.init_val = init_val
        self.false_alarm_prob = false_alarm_prob
        # pre-calculated v for detected or not detected targets
        self.v_for_1 = np.log(self.false_alarm_prob/(1-self.false_alarm_prob))
        self.v_for_0 = np.log((1-self.false_alarm_prob)/self.false_alarm_prob)
        

        self._left_lower_x = self.center_x - self.width / 2.0 * self.resolution
        self._left_lower_y = self.center_y - self.height / 2.0 * self.resolution

        self.ndata = self.width * self.height
        # this stores all data, {grid_inx: grid_value}
        self.non_empty_cell = dict()

    def _calc_xy_index_from_pos(self, pos, lower_pos, max_index):
        """Calculate the grid index by position
        """
        ind = int(np.floor((pos - lower_pos) / self.resolution))
        if not 0 <= ind <= max_index:
            # XXX may not need this warning
            logging.warning("Position not within the area")
        return ind

    def _calc_pos_from_xy_index(self, ind, lower_pos, _max_index):
        """Calculate the position by grid index
        """
        pos = ind*self.resolution+lower_pos+self.resolution/2.0
        return pos

    def get_value_from_xy_index(self, index):
        # type: (tuple) -> None
        """Get the value from given cell
        """
        return self.non_empty_cell[index]

    def get_xy_index_from_xy_pos(self, x_pos, y_pos):
        """Get grid index from position

        Args:
            x_pos ([type]): x position [m]
            y_pos ([type]): y position [m]

        Returns:
            tuple: the grid index of self.non_empty_cell
        """
        x_ind = self._calc_xy_index_from_pos(
            x_pos, self._left_lower_x, self.width)
        y_ind = self._calc_xy_index_from_pos(
            y_pos, self._left_lower_y, self.height)
        return tuple([int(x_ind), int(y_ind)])

    def get_xy_pos_from_xy_index(self, x_ind, y_ind):
        """get_xy_pos_from_xy_index
        """
        x_pos = self._calc_pos_from_xy_index(
            x_ind, self._left_lower_x, self.width)
        y_pos = self._calc_pos_from_xy_index(
            y_ind, self._left_lower_y, self.height)
        return tuple([x_pos, y_pos])

    def get_value_from_xy_pos(self, x_pos, y_pos):
        cell_ind = self.get_xy_index_from_xy_pos(x_pos, y_pos)
        return self.get_value_from_xy_index(cell_ind)

    def set_value_from_xy_index(self, index, val):
        """Stores the value in grid map

        Args:
            index (tuple): 2D tuple of x, y coordinates.
            val (float): Value that needs to be stored.
        """
        # If Q value after update is small enough to make the probability be zero,
        # it's safe to delete the cell for a better memory usage
        if val == 35.0:
            self.delete_value_from_xy_index(index)
        else:
            self.non_empty_cell[index] = val

    def delete_value_from_xy_index(self, index):
        """Delete the item from grid map

        Args:
            index (tuple): 2D tuple of x, y coordinates.
        """
        try:
            del self.non_empty_cell[index]
        except KeyError:
            logging.warning(f"{index} does't exist.")

    def generate_shareable_v(self, local_measurement):
        # type: (dict) -> dict
        """Generate the shareable information from local detection

        Args:
            local_measurement (dict): local detections

        Returns:
            dict: converted shareable detection info
        """
        meas_index = dict()
        for _target_id, meas in local_measurement.items():
            x_pos, y_pos, meas_confidence = meas
            point_ind = tuple(
                self.get_xy_index_from_xy_pos(x_pos, y_pos))
            # meas_index[point_ind] = meas_confidence
            meas_confidence = 1 - self.false_alarm_prob
            meas_index[point_ind] = np.log(
                self.false_alarm_prob/meas_confidence)
        # logging.debug(f"THE DETECTED: {meas_index}")
        return meas_index

    # def generate_zero_meas(self):
    #     def cut(x): return 1e-6 if x <= 1e-6 else 1 - \
    #         1e-6 if x >= 1-1e-6 else x
    #     meas_confidence = cut(np.random.normal(0.85, 0.1))
    #     x = np.log((1-self.false_alarm_prob)/(1-meas_confidence))
    #     return x

    def map_update(self, local_measurement, neighbor_measurement, N, d):
        """Update the probability map using measurements from local and neighbors

        Args:
            local_measurement (dict): Contains local detections like {id1:[x1, y1, confidence1], id2:[x2, y2, confidence2]}
            neighbor_measurement (dict): Contains neighbors' detections
            N (int): Number of all trackers (working on the same perimeter)
            d (int): Number of all neighbors
        """

        def bound_Q(Q):
            # 10 is big enough to make 1/(1+exp(10)) -> 0 and 1/(1+exp(-10)) -> 1
            return max(min(Q, 10), -10)

        # Get the weight of measurements
        weight_local = 1. - (d-1.)/N
        weight_neighbor = 1./N

        # Time decaying factor
        # NOTE Fine tune this param to get a good performance
        # alpha = 8
        # T = 0.1
        # decay_factor = np.exp(-alpha*T)
        decay_factor = 0.9
        # The diagram below shows the composition of the information for each update
        # ┌─────────────────────────────────────────────────────┐
        # │ Whole area                  .─────────.             │
        # │                          ,─'   Local   '─.          │
        # │             .─────────.,'   measurement   `.        │
        # │          ,─' Existing ╱'─.                  ╲       │
        # │        ,'      Cell  ;    `.                 :      │
        # │      ,'              │  2   `.      5        │      │
        # │     ;                │        :              │      │
        # │     ;                :       .─────────.     ;      │
        # │    ;                  ╲   ,─'  :        '─. ╱       │
        # │    │                   ╲,'  4  │       6   `.       │
        # │    │        1          ╱`.     │         ,'  ╲      │
        # │    :                  ;   '─.  ;      ,─'     :     │
        # │     :                 │      `───────'        │     │
        # │     :                 │  3    ;               │     │
        # │      ╲                :      ╱           7    ;     │
        # │       `.               ╲   ,'                ╱      │
        # │         `.              ╲,'   Neighbor      ╱       │
        # │           '─.         ,─'`.  measurement  ,'        │
        # │              `───────'     '─.         ,─'          │
        # │                               `───────'             │
        # └─────────────────────────────────────────────────────┘

        # update all existing grids (Area 1,2,3,4)
        for cell_ind in list(self.non_empty_cell):
            # Check if it's in area 2 or 4 (means we have local measurements about it)
            if cell_ind in local_measurement:
                v_local = local_measurement[cell_ind]
                del local_measurement[cell_ind]
            else:
                # If not, we believe there is no targets in that grid
                # v_local = self.generate_zero_meas()
                v_local = self.v_for_0

            if cell_ind in neighbor_measurement:
                v_neighbors = neighbor_measurement[cell_ind]
                del neighbor_measurement[cell_ind]
            else:
                v_neighbors = sum(
                    [self.v_for_0 for i in range(d)])

            Q = weight_local*(self.non_empty_cell[cell_ind] + v_local) + weight_neighbor * (
                d*self.non_empty_cell[cell_ind]+v_neighbors)
            self.set_value_from_xy_index(cell_ind, bound_Q(decay_factor * Q))

        # If got measurement for a new grid (Grids in area 5, 6, 7)
        else:
            # get the union set of all remaining measurements (Union of area 5, 6, 7)
            all_meas = set(list(local_measurement) +
                           list(neighbor_measurement))
            for cell_ind in all_meas:
                try:
                    v_local = local_measurement[cell_ind]
                except KeyError:
                    # v_local = self.generate_zero_meas()
                    v_local = self.v_for_0
                try:
                    v_neighbors = neighbor_measurement[cell_ind]
                except KeyError:
                    # v_neighbors = sum(
                    #     [self.generate_zero_meas() for i in range(d)])
                    v_neighbors = sum(
                        [self.v_for_0 for i in range(d)])
                Q = weight_local*(self.init_val + v_local) + weight_neighbor * (
                    d*self.init_val+v_neighbors)
                self.set_value_from_xy_index(
                    cell_ind, bound_Q(decay_factor * Q))

    def consensus(self, neighbors_map):
        # type: (dict) -> None
        """Merge neighbors map into local map and make a consensus

        Args:
            neighbors_map (dict): Contains all values from neighbors and have a count of it. Format: {(x, y):[value, count]}
        """
        for cell_ind, value in self.non_empty_cell.items():
            if cell_ind in neighbors_map.keys():
                # Calculate the average value of Q
                Q = (neighbors_map[cell_ind][0]+value) / \
                    (neighbors_map[cell_ind][1]+1)
                self.set_value_from_xy_index(cell_ind, Q)
                del neighbors_map[cell_ind]
        else:
            for cell_ind, value_and_count in neighbors_map.items():
                Q = value_and_count[0]/value_and_count[1]
                self.set_value_from_xy_index(cell_ind, Q)

    def convert_to_prob_map(self, threshold, normalization=False):
        """Convert log value to probability value [0~1]

        Args:
            threshold (float): Values higher than this will be returned
        """
        # logging.debug(f"{self.non_empty_cell}")
        lower_threshold = 0.05
        if threshold < 0.5:
            # shrink the lower threshold value
            lower_threshold *= threshold
            logging.warning(
                "Got probability threshold smaller than 0.5, it's not recommended.")
        self.prob_map = dict()

        max_prob = lower_threshold
        if normalization:
            # Generate the full prob map
            for cell_ind in list(self.non_empty_cell):
                value = self.non_empty_cell[cell_ind]
                # Decode the probability value
                prob = 1./(1.+np.exp(value))
                if prob > max_prob:
                    max_prob = prob
                self.prob_map[cell_ind] = prob
            # Normalize the whole map and delete data which is small enough
            for cell_ind in list(self.prob_map):
                factor = 1/max_prob
                normed_prob = factor*self.prob_map[cell_ind]
                if normed_prob >= threshold:
                    self.prob_map[cell_ind] = normed_prob
                else:
                    self.delete_value_from_xy_index(cell_ind)
                    del self.prob_map[cell_ind]
                    pass
                # if normed_prob <= lower_threshold*8:
                #     # keep some uncertainty between the lower and upper thresholds
                #     self.delete_value_from_xy_index(cell_ind)
        else:
            for cell_ind in list(self.non_empty_cell):
                value = self.non_empty_cell[cell_ind]
                # logging.debug(f"PROB: {value}")
                # Decode the probability value
                prob = 1./(1.+np.exp(value))
                if prob >= threshold:
                    self.prob_map[cell_ind] = prob
                    if prob >= 0.99999:
                        logging.warning(f"GOT 1!!! {prob} {value}")
                if prob < lower_threshold:
                    # logging.debug(f"Deleting {cell_ind},{prob}")
                    # keep some uncertainty between the lower and upper thresholds
                    self.delete_value_from_xy_index(cell_ind)
                    # pass
                    

    def get_target_est(self, threshold, normalization=False):
        """Get all targets' estimated position

        Args:
            threshold (float): Probability threshold value to filter out the targets

        Returns:
            list: Targets' position
        """
        # if normalization:
        #     logging.warning(
        #         "Using normalization for PorbMap, the real probability will be hidden.")
        self.convert_to_prob_map(threshold, normalization)
        targets_est = list(self.prob_map.keys())
        for i in range(len(targets_est)):
            # XXX since we don't need z-data, I put a placeholder here
            x, y = self.get_xy_pos_from_xy_index(
                targets_est[i][0], targets_est[i][1])
            targets_est[i] = [x, y, 150]
        return targets_est


class ProbMapData:
    def __init__(self):
        self.myid = -1
        self.type = 'n'
        self.grid_ind = list()
        self.values = list()
