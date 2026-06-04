# PyBulletの環境に関するクラス


# ライブラリの読み込み
import pybullet as p    # PyBullet


# 自作モジュールの読み込み
from constant import *  # 定数



class PyBulletEnvironment:
    # 定数の定義
    __PLANE_URDF = "plane.urdf"     # 地面に関する urdf ファイル
    
    # 各ロボットアームに応じた環境のベース位置
    __ENVIRONMENT_POS_2DOF = [0   ,  0,  0  ]   # 2軸ロボットアームの環境のベース位置
    __ENVIRONMENT_POS_3DOF = [1.5 ,  0, -0.5]   # 3軸ロボットアームの環境のベース位置
    __ENVIRONMENT_POS_6DOF = [1.75,  0,  0  ]   # 6軸ロボットアームの環境のベース位置
    
    
    def __init__(self, environment_urdf, n_robot_joint):
        """
        コンストラクタ

        パラメータ
            environment_urdf(str): 環境が保存されているファイル名
            n_robot_joint(int): ロボットの関節数 (グリッパーは含まない)
        """
        # 地面を読み込む (pybulletが提供している "plane.urdf" を読み込む)
        p.loadURDF(self.__PLANE_URDF)

        # ロボットアームに応じて，環境のベース位置を変える
        if n_robot_joint == DIMENTION_2D:
            # 2軸ロボットアーム
            basePosition = self.__ENVIRONMENT_POS_2DOF
        elif n_robot_joint == DIMENTION_3D:
            # 3軸ロボットアーム
            basePosition = self.__ENVIRONMENT_POS_3DOF
        elif n_robot_joint == DIMENTION_6D:
            # 6軸ロボットアーム
            basePosition = self.__ENVIRONMENT_POS_6DOF
        else:
            # 異常
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}")

        # 環境を読み込む (環境は常に移動しない)
        self.__environment_id = p.loadURDF(environment_urdf, basePosition=basePosition, useFixedBase=True)


    @property
    def environment_id(self):
        """
        __environment_idプロパティのゲッター
        """
        return self.__environment_id
