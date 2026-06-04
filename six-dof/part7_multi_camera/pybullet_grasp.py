import pybullet as p
import numpy as np

from constant import *


class PyBulletGraspObject:
    __GRASP_OBJ_POS_2DOF = [1.8 , 1.0, 0.05]
    __GRASP_OBJ_POS_3DOF = [1.25, 0.4, 0.55]
    __GRASP_OBJ_POS_6DOF = [1.5 , 0.4, 1.05]

    __GRASP_OBJ_OFFSET_2DOF = [-0.4 , 0      ]
    __GRASP_OBJ_OFFSET_3DOF = [-0.15, 0, 0.15]
    __GRASP_OBJ_OFFSET_6DOF = [-0.2 , 0, 0.2 ]

    __LATERAL_FRICTION  = 1.0
    __SPINNING_FRICTION = 1.0
    __ROLLING_FRICTION  = 1.0

    __GRASP_OBJ_OFFSET_PITCH = np.pi / 2


    def __init__(self, grasp_obj_urdf, n_robot_joint):
        if n_robot_joint == DIMENTION_2D:
            basePosition = self.__GRASP_OBJ_POS_2DOF
            self.__grasp_obj_offset = self.__GRASP_OBJ_OFFSET_2DOF
        elif n_robot_joint == DIMENTION_3D:
            basePosition = self.__GRASP_OBJ_POS_3DOF
            self.__grasp_obj_offset = self.__GRASP_OBJ_OFFSET_3DOF
        elif n_robot_joint == DIMENTION_6D:
            basePosition = self.__GRASP_OBJ_POS_6DOF
            self.__grasp_obj_offset = self.__GRASP_OBJ_OFFSET_6DOF
        else:
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}")

        self.__grasp_obj_id = p.loadURDF(grasp_obj_urdf, basePosition=basePosition)

        p.changeDynamics(self.__grasp_obj_id, -1,
                         lateralFriction=self.__LATERAL_FRICTION,
                         spinningFriction=self.__SPINNING_FRICTION,
                         rollingFriction=self.__ROLLING_FRICTION)

        grasp_pos, grasp_ori = self.get_grasp_pos(offset=False)

        self.__constraint_id = None
        self.set_constraint(grasp_pos, grasp_ori)

    def set_constraint(self, pos, ori):
        if self.__constraint_id is not None:
            raise ValueError("set_constraint() is double execution. please run release_constraint()")

        self.__constraint_id = p.createConstraint(
            self.__grasp_obj_id, -1, -1, -1,
            p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], pos,
            parentFrameOrientation=ori, childFrameOrientation=[0, 0, 0, 1]
        )

    def release_constraint(self):
        if self.__constraint_id is not None:
            p.removeConstraint(self.__constraint_id)
            self.__constraint_id = None

    def get_grasp_pos(self, offset=True, dim2=False):
        grasp_pos, grasp_ori = p.getBasePositionAndOrientation(self.__grasp_obj_id)
        grasp_pos = list(grasp_pos)

        roll, pitch, yaw = p.getEulerFromQuaternion(grasp_ori)
        pitch += self.__GRASP_OBJ_OFFSET_PITCH
        grasp_ori_rpy = [roll, pitch, yaw]

        if offset:
            grasp_pos = [pos + off for pos, off in zip(grasp_pos, self.__grasp_obj_offset)]

        if dim2:
            grasp_pos = grasp_pos[:DIMENTION_2D]

        return grasp_pos, grasp_ori_rpy

    @property
    def grasp_obj_id(self):
        return self.__grasp_obj_id
