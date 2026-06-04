import pybullet as p
import pybullet_data
import time
import numpy as np

from constant import *
from pybullet_rrt import RRTPyBullet


class MainPyBulletRobot:
    _PLANE_URDF     = "plane.urdf"
    _SIMULATION_SLEEP_TIME = 0.05
    _INTERFERENCE_MARGIN   = 0.1
    _PATH_PLAN_TIME = 100
    _N_MARGIN_MOVE  = 100


    def __init__(self, robot_urdf, environment_urdf, interpolation):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -GRABITY_VALUE)

        self._init_robot(robot_urdf, interpolation)
        self._init_environment(environment_urdf)
        self._rrt = None

    def _init_robot(self, robot_urdf, interpolation):
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        self._interpolation = interpolation
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        if not (self._n_joints == DIMENTION_2D or self._n_joints == DIMENTION_3D):
            raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

    def _init_environment(self, environment_urdf):
        p.loadURDF(self._PLANE_URDF)
        self._environment_id = p.loadURDF(environment_urdf, basePosition=[0, 0, 0], useFixedBase=True)


    def run(self, start_pos, end_pos, path_plan):
        p.setRealTimeSimulation(1)

        self._is_interference_start_end_pos(start_pos, end_pos)

        if path_plan == PATHPLAN.RRT.value:
            self._rrt = RRTPyBullet()
        else:
            raise ValueError(f"path_plan is abnormal. path_plan is {path_plan}")

        result = self._path_planning(start_pos, end_pos)

        if result:
            self._post_path_planning(start_pos, end_pos)

        self._rrt.save()

        for _ in range(self._N_MARGIN_MOVE):
            end_theta = self._convert_pos_to_theta(end_pos)
            self._set_joint(end_theta)
            time.sleep(self._SIMULATION_SLEEP_TIME)

        return result

    def _path_planning(self, start_pos, end_pos):
        result = False
        self._rrt.preparation(start_pos, end_pos, self._interpolation)

        start_time = time.time()
        while True:
            if (time.time() - start_time) >= self._PATH_PLAN_TIME:
                break

            new_node_pos, near_node_pos, near_node = self._rrt.expand_once(end_pos)
            if self._is_line_interference(new_node_pos, near_node_pos):
                continue
            if not self._rrt.add_node_and_chk_goal(end_pos, new_node_pos, near_node):
                continue
            if not self._is_line_interference(new_node_pos, end_pos):
                result = True
                break

        return result

    def _post_path_planning(self, start_pos, end_pos):
        self._rrt.fin_planning(start_pos, end_pos)

        theta = self._convert_pos_to_theta(start_pos)
        self._set_jump_joint(theta)

        for row_idx in range(self._rrt.pathes.shape[0]):
            next_theta = self._convert_pos_to_theta(self._rrt.pathes[row_idx])
            self._set_joint(next_theta)
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _set_jump_joint(self, thetas):
        for i in range(len(thetas)):
            p.resetJointState(bodyUniqueId=self._robot_id, jointIndex=i, targetValue=thetas[i])

    def _set_joint(self, thetas):
        for i in range(len(thetas)):
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id, jointIndex=i,
                controlMode=p.POSITION_CONTROL, targetPosition=thetas[i]
            )

    def _is_line_interference(self, pos1, pos2):
        is_interference = True
        if self._is_interference_pos(pos2):
            return is_interference
        if self._is_interference_pos(pos1):
            return is_interference

        theta = self._convert_pos_to_theta(pos2)
        self._set_joint(theta)
        time.sleep(self._SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference

    def _is_interference_start_end_pos(self, start_pos, end_pos):
        if self._is_interference_pos(start_pos):
            raise ValueError("start_pos is interference. change start_pos.")
        if self._is_interference_pos(end_pos):
            raise ValueError("end_pos is interference. change end_pos.")

    def _is_interference_pos(self, pos):
        is_interference = True
        theta = self._convert_pos_to_theta(pos)
        self._set_jump_joint(theta)
        time.sleep(self._SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference

    def convert_pos_to_theta(self, pos):
        if pos.shape[0] == DIMENTION_2D:
            pos = np.append(pos, 0.0)
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        return np.array(thetas)

    def _convert_pos_to_theta(self, pos):
        if self._interpolation == INTERPOLATION.POSITION.value:
            if pos.shape[0] == DIMENTION_2D:
                pos = np.append(pos, 0.0)
            thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
            return np.array(thetas)
        return np.copy(pos)
