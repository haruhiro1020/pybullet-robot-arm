from enum import Enum


DIMENTION_NONE  = -1
DIMENTION_2D    =  2
DIMENTION_3D    =  3

GRABITY_VALUE   = 9.81

MIN_SEED        = 0
MAX_SEED        = 2 ** 32 - 1

EPSILON         = 1e-6

RRT_NEAR_NODE_IDX       = -1

INITIAL_NODE_NEAR_NODE  = -1


class INTERPOLATION(Enum):
    NONE      = "none"
    JOINT     = "joint"
    POSITION  = "pos"


class PATHPLAN(Enum):
    RRT     = "rrt"


class ROBOTURDF(Enum):
    DOF2      = "robot_2dof.urdf"
    DOF2_HAND = "robot_2dof_hand.urdf"
    DOF3      = "robot_3dof.urdf"
    DOF3_HAND = "robot_3dof_hand.urdf"
