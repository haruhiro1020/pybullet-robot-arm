import pybullet as p
import numpy as np

from constant import *


class BaseGripper:
    _JOINT_CURRENT_VALUE_IDX = 0

    def __init__(self, robot_id, n_joint):
        self._robot_id = robot_id
        self._n_joint  = n_joint

    def run(self):
        raise InterruptedError("run() is necessary override.")


class ParallelGripper(BaseGripper):
    __GRIPPER_RIGHT_IDX =  -2
    __GRIPPER_LEFT_IDX  =  -1

    __LATERAL_FRICTION  = 1.0

    __CLOSE_VAL = 0.03
    __OPEN_VAL  = 0


    def __init__(self, robot_id, n_joint):
        super().__init__(robot_id, n_joint)
        self.__chg_dynamics()

    def __chg_dynamics(self):
        for idx in self.__get_gripper_right_left_idx():
            p.changeDynamics(self._robot_id, idx, lateralFriction=self.__LATERAL_FRICTION)

    def run(self, open=False, close=False):
        joint_values = self.__get_joint_values()

        if open:
            direction    = self.__get_move_direction(open=True)
            joint_values = direction * self.__OPEN_VAL
        elif close:
            direction    = self.__get_move_direction(open=False)
            joint_values = direction * self.__CLOSE_VAL

        self.__set_joint_values(joint_values)

    def __get_move_direction(self, open):
        move_direction = np.array([-1.0, 1.0])
        if not open:
            move_direction *= -1
        return move_direction

    def __get_gripper_right_left_idx(self):
        return [self._n_joint + self.__GRIPPER_RIGHT_IDX, self._n_joint + self.__GRIPPER_LEFT_IDX]

    def __set_joint_values(self, values):
        if values.shape[0] != DIMENTION_2D:
            raise ValueError(f"values'shape[0] is abnormal. values'shape[0] is {values.shape[0]}")

        for idx, gripper_idx in enumerate(self.__get_gripper_right_left_idx()):
            p.setJointMotorControl2(
                bodyIndex=self._robot_id,
                jointIndex=gripper_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=values[idx],
                positionGain=0.5,
                velocityGain=1.0
            )

    def __get_joint_values(self):
        joint_values = []
        for gripper_idx in self.__get_gripper_right_left_idx():
            joint_state = p.getJointState(bodyUniqueId=self._robot_id, jointIndex=gripper_idx)
            joint_values.append(joint_state[self._JOINT_CURRENT_VALUE_IDX])
        return np.array(joint_values)
