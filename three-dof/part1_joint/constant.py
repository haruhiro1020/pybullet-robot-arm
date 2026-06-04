from enum import Enum


DIMENTION_NONE  = -1
DIMENTION_2D    =  2
DIMENTION_3D    =  3

GRABITY_VALUE   = 9.81


class INTERPOLATION(Enum):
    JOINT     = "joint"
    POSITION  = "pos"
