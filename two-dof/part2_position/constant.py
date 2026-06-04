# 複数ファイルで使用する定数の定義
from enum import Enum


# 次元数を定義
DIMENTION_2D    =  2    # 2次元


# 重力
GRABITY_VALUE   = 9.81


# 補間方法の定義
class INTERPOLATION(Enum):
    """
    補間方法
    """
    JOINT     = "joint"     # 関節補間
    POSITION  = "pos"       # 位置補間


