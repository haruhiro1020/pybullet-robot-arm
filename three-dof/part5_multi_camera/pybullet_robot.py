import pybullet as p
import numpy as np

from constant import *
from pybullet_gripper import ParallelGripper


class _PyBulletRobot:
    _DIM_JOINT          = DIMENTION_NONE
    _DIM_POSE           = DIMENTION_NONE
    _N_HAND_JOINT       = 4
    _N_HAND_JOINT_NOT_FIXED = 2
    _WEIGHT_JOINT       = None
    _JOINT_LIMIT_LOW_IDX = 8
    _JOINT_LIMIT_UP_IDX  = 9


    def __init__(self, robot_id, interpolation, hand):
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        self._robot_id      = robot_id
        self._interpolation = interpolation
        self._n_joints      = p.getNumJoints(robot_id) - 1

        if hand:
            if self._n_joints != (self._DIM_JOINT + self._N_HAND_JOINT):
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")
            self._hand = ParallelGripper(self._robot_id, self._n_joints)
        else:
            if self._n_joints != self._DIM_JOINT:
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")
            self._hand = None

        self.__joint_limit()

    def __joint_limit(self):
        joint_limit = []
        for joint_index in range(self._DIM_JOINT):
            joint_info = p.getJointInfo(self._robot_id, joint_index)
            lower = joint_info[self._JOINT_LIMIT_LOW_IDX]
            upper = joint_info[self._JOINT_LIMIT_UP_IDX]
            joint_limit.append((lower, upper))
        self._joints_limit = np.array(joint_limit)

    @property
    def robot_id(self):
        return self._robot_id

    @property
    def joints_limit(self):
        return self._joints_limit

    @property
    def weight_joint(self):
        return self._WEIGHT_JOINT

    @property
    def interpolation(self):
        return self._interpolation

    def _chk_pos_dim(self, pos):
        if len(pos) != self._DIM_POSE:
            raise ValueError(f"pos's shape is abnormal. pos'size is {len(pos)}")

    def _chk_thetas_dim(self, thetas):
        if len(thetas) != self._DIM_JOINT:
            raise ValueError(f"theta's shape is abnormal. thetas'size is {len(thetas)}")

    def set_joint(self, thetas):
        self._chk_thetas_dim(thetas)
        for i in range(len(thetas)):
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id, jointIndex=i,
                controlMode=p.POSITION_CONTROL, targetPosition=thetas[i]
            )

    def set_jump_joint(self, thetas):
        self._chk_thetas_dim(thetas)
        for i in range(thetas.shape[0]):
            p.resetJointState(bodyUniqueId=self._robot_id, jointIndex=i, targetValue=thetas[i])

    def convert_pos_to_theta(self, pos, force=False):
        raise InterruptedError("convert_pos_to_theta() is necessary override.")

    def run_gripper(self, open=False, close=False):
        if self._hand is None:
            return
        self._hand.run(open, close)


class _PyBullet3DoFRobot(_PyBulletRobot):
    _DIM_JOINT = DIMENTION_3D
    _DIM_POSE  = DIMENTION_3D


    def __init__(self, robot_id, interpolation, hand):
        super().__init__(robot_id, interpolation, hand)

    def __inverse_kinematics(self, pos):
        self._chk_pos_dim(pos)
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        return np.array(thetas)

    def convert_pos_to_theta(self, pos, force=False):
        if force:
            thetas = self.__inverse_kinematics(pos)
        else:
            if self._interpolation == INTERPOLATION.POSITION.value:
                thetas = self.__inverse_kinematics(pos)
            else:
                thetas = np.copy(pos)

        if self._hand is not None:
            thetas = thetas[:self._DIM_JOINT]

        return thetas


class PyBulletRobotController:
    __ROBOT_BASE_POSITION = [0, 0, 0]


    def __init__(self, n_robot_joint, interpolation, hand):
        if n_robot_joint == DIMENTION_3D:
            robot_urdf = ROBOTURDF.DOF3_HAND.value if hand else ROBOTURDF.DOF3.value
            robot_cls  = _PyBullet3DoFRobot
        else:
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}.")

        robot_id = p.loadURDF(robot_urdf, basePosition=self.__ROBOT_BASE_POSITION, useFixedBase=True)

        self.__robot         = robot_cls(robot_id, interpolation, hand)
        self.__n_robot_joint = n_robot_joint

    @property
    def robot_id(self):
        return self.__robot.robot_id

    @property
    def joints_limit(self):
        return self.__robot.joints_limit

    @property
    def n_robot_joint(self):
        return self.__n_robot_joint

    @property
    def weight_joint(self):
        return self.__robot.weight_joint

    @property
    def interpolation(self):
        return self.__robot.interpolation

    def set_joint(self, thetas):
        self.__robot.set_joint(thetas)

    def set_jump_joint(self, thetas):
        self.__robot.set_jump_joint(thetas)

    def convert_pos_to_theta(self, pos, force=False):
        return self.__robot.convert_pos_to_theta(pos, force)

    def run_gripper(self, open=False, close=False):
        self.__robot.run_gripper(open=open, close=close)
