import pybullet as p
import time

from constant import *
from pybullet_robot import PyBulletRobotController
from pybullet_environment import PyBulletEnvironment


class PyBulletInteference:
    __SIMULATION_SLEEP_TIME = 0.05
    __INTERFERENCE_MARGIN   = 0.15


    def __init__(self, robot: PyBulletRobotController, environment: PyBulletEnvironment):
        self.__robot       = robot
        self.__environment = environment

    def is_line_interference(self, pos1, pos2):
        is_interference = True

        if self.is_interference_pos(pos2):
            return is_interference
        if self.is_interference_pos(pos1):
            return is_interference

        theta = self.__robot.convert_pos_to_theta(pos2)
        self.__robot.set_joint(theta)
        self.__robot.run_gripper(open=True)
        time.sleep(self.__SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self.__robot.robot_id, self.__environment.environment_id, self.__INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference

    def is_interference_pos(self, pos):
        is_interference = True

        theta = self.__robot.convert_pos_to_theta(pos)
        self.__robot.set_jump_joint(theta)
        self.__robot.run_gripper(open=True)
        time.sleep(self.__SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self.__robot.robot_id, self.__environment.environment_id, self.__INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference
