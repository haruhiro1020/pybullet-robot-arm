# 複数ファイルで使用する定数の定義
from enum import Enum
from enum import auto


# 次元数を定義
DIMENTION_NONE  = -1    # 未定義
DIMENTION_2D    =  2    # 2次元
DIMENTION_3D    =  3    # 3次元
DIMENTION_6D    =  6    # 6次元

# 重力
GRABITY_VALUE   = 9.81

# シード値の最小値と最大値
MIN_SEED        = 0
MAX_SEED        = 2 ** 32 - 1  # 4バイト (uint) の最大値

# 0割を防ぐための定数
EPSILON         = 1e-6

# ノードの最短ノード要素とコスト要素を定義
# RRT
RRT_NEAR_NODE_IDX           = -1    # RRTでの最短ノード要素
# RRT-Connect
RRTCONNECT_NEAR_NODE_IDX    = -1    # RRT-Connectでの最短ノード要素
# RRT*
RRTSTAR_NEAR_NODE_IDX       = -2    # RRT*での最短ノード要素
RRTSTAR_COST_IDX            = -1    # RRT*でのコスト要素

# ツリーの初期ノードの親ノード
INITIAL_NODE_NEAR_NODE  = -1    # 初期ノードに親ノードが存在しないから-1


# RRT-Connectの状態を定義
class RRTCONNECTSTATE(Enum):
    """
    RRT-Connectの状態
    """
    STREE_RAND      = 0         # 始点ツリーにランダム点を追加
    STREE_TO_ETREE  = auto()    # 始点ツリーから終点ツリーへノードを伸ばす
    ETREE_RAND      = auto()    # 終点ツリーにランダム点を追加
    ETREE_TO_STREE  = auto()    # 終点ツリーから始点ツリーへノードを伸ばす


# 補間方法の定義
class INTERPOLATION(Enum):
    """
    補間方法
    """
    NONE      = "none"      # 未定義
    JOINT     = "joint"     # 関節補間
    POSITION  = "pos"       # 位置補間


# RRTによる経路生成手法の定義
class PATHPLAN(Enum):
    """
    経路生成手法
    """
    RRT         = "rrt"             # RRTによる経路生成
    RRTCONNECT  = "rrt-connect"     # RRT-Connectによる経路生成
    RRTSTAR     = "rrt-star"        # RRT*による経路生成


# ロボットアームが保存されている URDF ファイル名
class ROBOTURDF(Enum):
    """
    ロボットのURDFファイル名
    """
    # 2軸ロボットアーム
    DOF2 = "robot_2dof.urdf"            # ハンド(グリッパ)なし
    DOF2_HAND = "robot_2dof_hand.urdf"  # ハンド(グリッパ)付き

    # 3軸ロボットアーム
    DOF3 = "robot_3dof.urdf"            # ハンド(グリッパ)なし
    DOF3_HAND = "robot_3dof_hand.urdf"  # ハンド(グリッパ)付き

    # 6軸ロボットアーム
    DOF6 = "robot_6dof.urdf"            # ハンド(グリッパ)なし
    DOF6_HAND = "robot_6dof_hand.urdf"  # ハンド(グリッパ)付き


# カメラが保存されている URDF ファイル名
class CAMERAURDF(Enum):
    """
    カメラのURDFファイル名
    """
    # X軸方向に向いているカメラ
    RIGHT2LEFT = "camera_right_to_left.urdf"    # 左向き(-X方向)のカメラ
    LEFT2RIGHT = "camera_left_to_right.urdf"    # 右向き(+X方向)のカメラ

    # Y軸方向に向いているカメラ
    FRONT2BACK = "camera_front_to_back.urdf"    # 奥向き(+Y方向)のカメラ
    BACK2FRONT = "camera_back_to_front.urdf"    # 手前向き(-Y方向)のカメラ

    # Z軸方向に向いているカメラ
    UP2DOWN = "camera_up_to_down.urdf"  # 下向き(-Z方向)のカメラ


# カメラ数を定義する定数
class CAMERANUM(Enum):
    """
    カメラ数
    """
    SINGLE = 1          # 1つのカメラ
    MULTI  = 5          # 複数のカメラ
