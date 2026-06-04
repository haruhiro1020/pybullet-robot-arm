from enum import Enum
from enum import auto


DIMENTION_NONE  = -1
DIMENTION_2D    =  2
DIMENTION_3D    =  3
DIMENTION_6D    =  6

GRABITY_VALUE   = 9.81

MIN_SEED        = 0
MAX_SEED        = 2 ** 32 - 1

EPSILON         = 1e-6

RRT_NEAR_NODE_IDX           = -1
RRTCONNECT_NEAR_NODE_IDX    = -1
RRTSTAR_NEAR_NODE_IDX       = -2
RRTSTAR_COST_IDX            = -1


class RRTCONNECTSTATE(Enum):
    STREE_RAND      = 0
    STREE_TO_ETREE  = auto()
    ETREE_RAND      = auto()
    ETREE_TO_STREE  = auto()


INITIAL_NODE_NEAR_NODE  = -1


class INTERPOLATION(Enum):
    NONE      = "none"
    JOINT     = "joint"
    POSITION  = "pos"


class PATHPLAN(Enum):
    RRT         = "rrt"
    RRTCONNECT  = "rrt-connect"
    RRTSTAR     = "rrt-star"


class ROBOTURDF(Enum):
    DOF2      = "robot_2dof.urdf"
    DOF2_HAND = "robot_2dof_hand.urdf"
    DOF3      = "robot_3dof.urdf"
    DOF3_HAND = "robot_3dof_hand.urdf"
    DOF6      = "robot_6dof.urdf"
    DOF6_HAND = "robot_6dof_hand.urdf"


class CAMERAURDF(Enum):
    RIGHT2LEFT = "camera_right_to_left.urdf"
    LEFT2RIGHT = "camera_left_to_right.urdf"
    FRONT2BACK = "camera_front_to_back.urdf"
    BACK2FRONT = "camera_back_to_front.urdf"
    UP2DOWN    = "camera_up_to_down.urdf"


class CAMERANUM(Enum):
    SINGLE = 1
    MULTI  = 5
