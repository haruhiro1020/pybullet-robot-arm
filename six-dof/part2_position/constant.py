from enum import Enum


DIMENTION_NONE  = -1
DIMENTION_2D    =  2
DIMENTION_3D    =  3
DIMENTION_6D    =  6

GRABITY_VALUE   = 9.81
EPSILON         = 1e-6


class INTERPOLATION(Enum):
    JOINT     = "joint"
    POSITION  = "pos"
