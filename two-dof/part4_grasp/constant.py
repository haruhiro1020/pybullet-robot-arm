# 複数ファイルで使用する定数の定義
from enum import Enum
from enum import auto


# 次元数を定義
DIMENTION_NONE  = -1    # 未定義
DIMENTION_2D    =  2    # 2次元

# 重力
GRABITY_VALUE   = 9.81

# シード値の最小値と最大値
MIN_SEED        = 0
MAX_SEED        = 2 ** 32 - 1  # 4バイト (uint) の最大値

# 0割を防ぐための定数
EPSILON         = 1e-6

# ノードの最短ノード要素とコスト要素を定義
# RRT
RRT_NEAR_NODE_IDX       = -1    # RRTでの最短ノード要素

# ツリーの初期ノードの親ノード
INITIAL_NODE_NEAR_NODE  = -1    # 初期ノードに親ノードが存在しないから-1

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
    RRT     = "rrt"     # RRTによる経路生成

# ロボット手先位置のキーボード割り当て
class KEYBOARD(Enum):
    """
    キーボード割り当て
        (ord()により，文字からUnicode番号へ変換)
    """
    PLUS_X  = ord("d")      # +x方向へ移動
    MINUS_X = ord("a")      # -x方向へ移動
    PLUS_Y  = ord("z")      # +y方向へ移動
    MINUS_Y = ord("x")      # -y方向へ移動
    PLUS_Z  = ord("e")      # +z方向へ移動
    MINUS_Z = ord("q")      # -z方向へ移動
    
    PLUS_ROLL   = ord("d")  # +Roll(X)方向へ移動
    MINUS_ROLL  = ord("a")  # -Roll(X)方向へ移動
    PLUS_PITCH  = ord("w")  # +Pitch(Y)方向へ移動
    MINUS_PITCH = ord("s")  # -Pitch(Y)方向へ移動
    PLUS_YAW    = ord("e")  # +YAW(Z)方向へ移動
    MINUS_YAW   = ord("q")  # -YAW(Z)方向へ移動
    
    GRIP_OPEN   = ord("f")  # グリッパーのオープン
    GRIP_CLOSE  = ord("r")  # グリッパーのクローズ
