from enum import Enum
import numpy as np


DIMENTION_NONE  = -1
DIMENTION_2D    =  2
DIMENTION_3D    =  3
DIMENTION_6D    =  6

GRABITY_VALUE   = 9.81
EPSILON         = 1e-6

RRT_NEAR_NODE_IDX       = -1
INITIAL_NODE_NEAR_NODE  = -1


class INTERPOLATION(Enum):
    NONE      = "none"
    JOINT     = "joint"
    POSITION  = "pos"


class PATHPLAN(Enum):
    RRT     = "rrt"
