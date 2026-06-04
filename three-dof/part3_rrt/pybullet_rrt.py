import numpy as np
import os

from constant import *


class Tree:
    def __init__(self, near_node_idx):
        self._nodes         = []
        self._near_node_idx = near_node_idx

    @property
    def nodes(self):
        return self._nodes

    def reset(self):
        if len(self._nodes) != 0:
            del self._nodes
        self._nodes = []

    def add_node(self, node):
        if len(self._nodes) == 0:
            self._nodes = node
        else:
            self._nodes = np.append(self._nodes, node, axis=0)

    def _chk_node_exist(self):
        if len(self._nodes) == 0:
            raise ValueError("self._nodes is not exist")

    def get_near_node(self, pos):
        self._chk_node_exist()
        nodes_pos  = self._nodes[:, :self._near_node_idx]
        difference = nodes_pos - pos
        distance   = np.linalg.norm(difference, axis=1)
        return np.argmin(distance)

    def get_near_node_list(self, pos, radius):
        self._chk_node_exist()
        all_node_pos = self._nodes[:, :self._near_node_idx]
        difference   = all_node_pos - pos
        distance     = np.linalg.norm(difference, axis=1)
        return [idx for idx, dist in enumerate(distance) if dist <= radius]


class RRTPyBullet:
    _FILE_NAME_PATHES     = f"{PATHPLAN.RRT.value}_pathes.csv"
    _FILE_NAME_START_TREE = f"{PATHPLAN.RRT.value}_start_tree.csv"

    _NODE_NEAR_NODE_IDX = RRT_NEAR_NODE_IDX

    _MOVING_VALUE_JOINT = 0.1
    _MOVING_VALUE_POS   = 0.2
    _STRICT_PLANNING_ROB_JOINT = np.pi / 2
    _STRICT_PLANNING_ROB_POS   = 1.0

    _GOAL_SAMPLE_RATE = 0.1


    def __init__(self):
        self._name        = PATHPLAN.RRT.value
        self._pathes      = []
        self._start_tree  = Tree(self._NODE_NEAR_NODE_IDX)
        self._interpolation = INTERPOLATION.NONE.value
        self._moving_value  = self._MOVING_VALUE_JOINT
        self._dim = DIMENTION_NONE

    @property
    def name(self):
        return self._name

    @property
    def pathes(self):
        return self._pathes


    def expand_once(self, end_pos):
        random_pos    = self._get_random_pos(end_pos)
        near_node     = self._start_tree.get_near_node(random_pos)
        near_node_pos = self._start_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
        new_node_pos  = self._calc_new_pos(random_pos, near_node_pos)
        return new_node_pos, near_node_pos, near_node

    def add_node_and_chk_goal(self, end_pos, node_pos, near_node):
        is_successful = False
        self._add_node_start_tree(node_pos, near_node)
        if self._chk_end_pos_dist(node_pos, end_pos):
            is_successful = True
        return is_successful

    def _add_node_start_tree(self, pos, near_node):
        node = np.append(pos, near_node).reshape(1, -1)
        self._start_tree.add_node(node)

    def _chk_end_pos_dist(self, pos, end_pos):
        dist = np.linalg.norm(end_pos - pos)
        return dist <= self._moving_value

    def fin_planning(self, start_pos, end_pos):
        revers_path = end_pos.reshape(1, -1)
        near_node   = -1

        while True:
            node      = self._start_tree.nodes[near_node]
            pos       = node[:self._NODE_NEAR_NODE_IDX].reshape(1, -1)
            near_node = int(node[self._NODE_NEAR_NODE_IDX])
            revers_path = np.append(revers_path, pos, axis=0)
            if near_node == INITIAL_NODE_NEAR_NODE:
                break

        self._pathes = revers_path[::-1]
        self._add_node_start_tree(end_pos, self._start_tree.nodes.shape[0] - 1)

    def _calc_new_pos(self, random_pos, near_node_pos):
        direction      = random_pos - near_node_pos
        norm_direction = direction / (np.linalg.norm(direction) + EPSILON)
        return near_node_pos + norm_direction * self._moving_value

    def _get_random_pos(self, target_pos):
        if np.random.rand() < self._GOAL_SAMPLE_RATE:
            return target_pos
        return np.random.uniform(self._strict_min_pos, self._strict_max_pos)


    def preparation(self, start_pos, end_pos, interpolation):
        self._reset()

        if np.size(start_pos) != np.size(end_pos):
            raise ValueError("start_pos_dim and end_pos_dim are not matched.")

        self._dim = np.size(start_pos)
        self._set_interpolation(interpolation)
        self._add_node_start_tree(start_pos, INITIAL_NODE_NEAR_NODE)
        self._strict_planning_pos(start_pos, end_pos)
        self._make_path_plan_folder(interpolation)
        self._reset_folder(interpolation)

    def _reset(self):
        self._start_tree.reset()
        if len(self._pathes) != 0:
            del self._pathes
        self._pathes        = []
        self._interpolation = INTERPOLATION.NONE.value
        self._dim           = DIMENTION_NONE

    def _set_interpolation(self, interpolation):
        if interpolation == INTERPOLATION.POSITION.value:
            self._moving_value = self._MOVING_VALUE_POS
        elif interpolation == INTERPOLATION.JOINT.value:
            self._moving_value = self._MOVING_VALUE_JOINT
        else:
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")
        self._interpolation = interpolation

    def _strict_planning_pos(self, start_pos, end_pos):
        all_pos = np.array([start_pos, end_pos])
        min_pos = np.min(all_pos, axis=0)
        max_pos = np.max(all_pos, axis=0)

        if self._interpolation == INTERPOLATION.POSITION.value:
            strict_planning_pos = self._STRICT_PLANNING_ROB_POS
        else:
            strict_planning_pos = self._STRICT_PLANNING_ROB_JOINT

        self._strict_min_pos = min_pos - strict_planning_pos
        self._strict_max_pos = max_pos + strict_planning_pos


    def _make_folder(self, folder_name):
        os.makedirs(folder_name, exist_ok=True)

    def _reset_folder(self, interpolation):
        for folder_name in self._get_path_plan_folders(interpolation):
            for entry in os.listdir(folder_name):
                full_path = os.path.join(folder_name, entry)
                if os.path.isfile(full_path) or os.path.islink(full_path):
                    os.remove(full_path)

    def _get_path_plan_folders(self, interpolation):
        return [os.path.join(self._name, interpolation)]

    def _make_path_plan_folder(self, interpolation):
        for folder_name in self._get_path_plan_folders(interpolation):
            self._make_folder(folder_name)

    def save(self):
        self._save_numpy_data_to_txt(self._pathes, self._FILE_NAME_PATHES)
        self._save_numpy_data_to_txt(self._start_tree.nodes, self._FILE_NAME_START_TREE)

    def _save_numpy_data_to_txt(self, data, file_name):
        if len(data) == 0 or not file_name:
            return
        full_path = f"{self._name}/{self._interpolation}/{file_name}"
        np.savetxt(full_path, data)
