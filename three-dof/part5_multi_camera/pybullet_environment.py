import pybullet as p

from constant import *


class PyBulletEnvironment:
    __PLANE_URDF = "plane.urdf"

    __ENVIRONMENT_POS_2DOF = [0   ,  0,  0  ]
    __ENVIRONMENT_POS_3DOF = [1.5 ,  0, -0.5]
    __ENVIRONMENT_POS_6DOF = [1.75,  0,  0  ]


    def __init__(self, environment_urdf, n_robot_joint):
        p.loadURDF(self.__PLANE_URDF)

        if n_robot_joint == DIMENTION_2D:
            basePosition = self.__ENVIRONMENT_POS_2DOF
        elif n_robot_joint == DIMENTION_3D:
            basePosition = self.__ENVIRONMENT_POS_3DOF
        elif n_robot_joint == DIMENTION_6D:
            basePosition = self.__ENVIRONMENT_POS_6DOF
        else:
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}")

        self.__environment_id = p.loadURDF(environment_urdf, basePosition=basePosition, useFixedBase=True)

    @property
    def environment_id(self):
        return self.__environment_id
