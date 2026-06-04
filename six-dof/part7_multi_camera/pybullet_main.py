import pybullet as p
import pybullet_data
import time
import numpy as np

from constant import *
from pybullet_rrt import PyBulletRRTController
from pybullet_environment import PyBulletEnvironment
from pybullet_grasp import PyBulletGraspObject
from pybullet_robot import PyBulletRobotController
from pybullet_interference import PyBulletInteference
from pybullet_camera import PyBulletCameraContoller


class MainPyBulletRobot:
    __SIMULATION_SLEEP_TIME = 0.05
    __N_MARGIN_MOVE         = 20

    # 6軸用オフセット
    __GRASP_OBJ_OFFSET_6DOF = np.array([-0.2, 0, 0.2])


    def __init__(self, interpolation, n_robot_joint, environment_urdf, grasp_urdf, n_cameras, hand=False):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -GRABITY_VALUE)

        self.__robot        = PyBulletRobotController(n_robot_joint, interpolation, hand)
        self.__environment  = PyBulletEnvironment(environment_urdf, self.__robot.n_robot_joint)
        self.__grasp_obj    = PyBulletGraspObject(grasp_urdf, self.__robot.n_robot_joint)
        self.__interference = PyBulletInteference(self.__robot, self.__environment)
        self.__camera       = PyBulletCameraContoller(n_cameras)
        self.__rrt          = None
        self.__n_robot_joint = n_robot_joint


    def run(self, start_pos, path_plan):
        p.setRealTimeSimulation(1)

        end_pos = self.__get_grasp_pos_from_camera()
        print(f"end_pos = {end_pos}")

        self.__rrt = PyBulletRRTController(path_plan)

        result = self.__rrt.planning(
            start_pos, end_pos,
            self.__robot.interpolation,
            self.__interference,
            self.__robot.joints_limit,
            self.__robot.weight_joint
        )

        print(f"planning result = {result}")

        if result:
            self.__post_path_planning(start_pos, end_pos)

        self.__rrt.save()
        self.__exec_margin_time(end_pos)

        return result

    def __get_grasp_pos_from_camera(self):
        pos = self.__camera.get_pos(self.__grasp_obj.grasp_obj_id)

        if self.__n_robot_joint == DIMENTION_6D:
            # 6軸: オフセットを加えて3D+姿勢として返す
            pos_3d = pos + self.__GRASP_OBJ_OFFSET_6DOF
            # デフォルト姿勢を付加
            orientation = np.array([np.pi/4, np.pi/2, 0])
            pos = np.concatenate([pos_3d, orientation])
        else:
            pos = pos[:DIMENTION_2D]

        if self.__robot.interpolation == INTERPOLATION.JOINT.value:
            pos = self.__robot.convert_pos_to_theta(pos)

        return pos

    def __exec_margin_time(self, end_pos):
        end_theta = self.__robot.convert_pos_to_theta(end_pos)

        for _ in range(self.__N_MARGIN_MOVE):
            self.__robot.set_joint(end_theta)
            self.__robot.run_gripper(open=True)
            time.sleep(self.__SIMULATION_SLEEP_TIME)

        self.__grasp_obj.release_constraint()
        time.sleep(self.__SIMULATION_SLEEP_TIME)

        for _ in range(self.__N_MARGIN_MOVE):
            self.__robot.set_joint(end_theta)
            self.__robot.run_gripper(close=True)
            time.sleep(self.__SIMULATION_SLEEP_TIME)

    def __post_path_planning(self, start_pos, end_pos):
        theta = self.__robot.convert_pos_to_theta(start_pos)
        self.__robot.set_jump_joint(theta)
        self.__robot.run_gripper(open=True)

        for row_idx in range(self.__rrt.pathes.shape[0]):
            next_theta = self.__robot.convert_pos_to_theta(self.__rrt.pathes[row_idx])
            self.__robot.set_joint(next_theta)
            self.__robot.run_gripper(open=True)
            time.sleep(self.__SIMULATION_SLEEP_TIME)

    def convert_pos_to_theta(self, pos):
        return self.__robot.convert_pos_to_theta(pos, force=True)
