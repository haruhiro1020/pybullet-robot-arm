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
