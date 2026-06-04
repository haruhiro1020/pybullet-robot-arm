# PyBulletの把持物体に関するクラス


# ライブラリの読み込み
import pybullet as p    # PyBullet
import numpy as np      # 数値計算


# 自作モジュールの読み込み
from constant import *  # 定数



class PyBulletGraspObject:
    # 定数の定義
    # 各ロボットアームに応じた把持物体のベース位置
    __GRASP_OBJ_POS_2DOF = [1.8 , 1.0, 0.05] # 2軸ロボットアームの把持物体のベース位置
    __GRASP_OBJ_POS_3DOF = [1.25, 0.4, 0.55] # 3軸ロボットアームの把持物体のベース位置
    __GRASP_OBJ_POS_6DOF = [1.5 , 0.4, 1.05] # 6軸ロボットアームの把持物体のベース位置
    
    # 各ロボットアームに応じた把持物体のオフセット位置
    __GRASP_OBJ_OFFSET_2DOF = [-0.4 , 0      ]   # 2軸ロボットアームの把持物体のオフセット
    __GRASP_OBJ_OFFSET_3DOF = [-0.15, 0, 0.15]   # 3軸ロボットアームの把持物体のオフセット
    __GRASP_OBJ_OFFSET_6DOF = [-0.2 , 0, 0.2 ]   # 6軸ロボットアームの把持物体のオフセット
    
    # 把持物体のパラメータ
    __LATERAL_FRICTION  = 1.0    # 床との摩擦係数
    __SPINNING_FRICTION = 1.0    # 回転摩擦係数
    __ROLLING_FRICTION  = 1.0    # 転がり摩擦係数
    
    __GRASP_OBJ_OFFSET_PITCH = np.pi / 2
    
    
    def __init__(self, grasp_obj_urdf, n_robot_joint):
        """
        コンストラクタ

        パラメータ
            grasp_obj_urdf(str): 把持物体が保存されているファイル名
            n_robot_joint(int): ロボットの関節数 (グリッパーは含まない)
        """
        # ロボットアームに応じて，把持物体のベース位置・オフセットを変える
        if n_robot_joint == DIMENTION_2D:
            # 2軸ロボットアーム
            basePosition = self.__GRASP_OBJ_POS_2DOF
            self.__grasp_obj_offset = self.__GRASP_OBJ_OFFSET_2DOF
        else:
            # 異常
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}")

        # 把持物体を読み込む
        self.__grasp_obj_id = p.loadURDF(grasp_obj_urdf, basePosition=basePosition)
        print(f"self.__grasp_obj_id = {self.__grasp_obj_id}")

        # 把持対象物に摩擦を付与する
        p.changeDynamics(self.__grasp_obj_id,   # 把持物体ID
                        -1,                     # ベースに対して
                        lateralFriction=self.__LATERAL_FRICTION,     # 床との摩擦係数
                        spinningFriction=self.__SPINNING_FRICTION,   # 回転摩擦係数
                        rollingFriction=self.__ROLLING_FRICTION)     # 転がり摩擦

        # 把持物体の位置・姿勢を取得
        grasp_pos, grasp_ori = self.get_grasp_pos(offset=False)

        # 把持物体に拘束条件を付与
        self.__constraint_id = None
        self.set_constraint(grasp_pos, grasp_ori)


    def set_constraint(self, pos, ori):
        """
        拘束条件の設定

        パラメータ
            pos(list): 拘束したい位置
            ori(list): 拘束したい姿勢
        """
        if self.__constraint_id is not None:
            # 拘束条件を2重に設定しようとしている
            raise ValueError("set_constrant() is double exection. please run release_constraint()")

        self.__constraint_id = p.createConstraint(
                            self.__grasp_obj_id,    # 親番号(拘束したい対象物ID)
                            -1,                     # 親リンクの要素番号("-1"はベース)
                            -1,                     # 子番号("-1"はなし)
                            -1,                     # 子リンクの要素番号("-1"はベース)
                            p.JOINT_FIXED,          # 関節タイプ(今回は固定"JOINT_FIXED")
                            [0, 0, 0],              # 関節軸
                            [0, 0, 0],              # 親の中心からの位置 
                            pos,                    # 子の中心からの位置 (今回は子を設定していないから，ワールド座標系から見た関節位置)
                            parentFrameOrientation=ori,         # 親の中心からの姿勢
                            childFrameOrientation=[0, 0, 0, 1]) # 子の中心からの姿勢 (今回は，子を設定していないから，ワールド座標系から見た姿勢)

    def release_constraint(self):
        """
        拘束条件の解除
        """
        if self.__constraint_id is not None:
            # 拘束条件を解除
            p.removeConstraint(self.__constraint_id)
            self.__constraint_id = None

    def get_grasp_pos(self, offset=True, dim2=False):
        """
        把持対象物の位置・姿勢を取得

        パラメータ
            offset(bool): 把持対象物へのオフセットを設定するかどうか
            dim2(bool): 2次元位置として取得するかどうか

        戻り値
            list: 把持対象物の位置 [m]
            list: 把持対象物の姿勢 (ロール・ピッチ・ヨー [rad])
        """
        # 把持対象物の位置[m]・姿勢[rad]を取得
        grasp_pos, grasp_ori = p.getBasePositionAndOrientation(self.__grasp_obj_id)
        # 位置をタプルからリストへ変換
        grasp_pos = list(grasp_pos)

        # 姿勢をクォータニオンからロール・ピッチ・ヨーへ変換
        roll, pitch, yaw = p.getEulerFromQuaternion(grasp_ori)
        # ピッチ角を90度回転させる → 把持対象物の正面(x方向)から把持したいから
        pitch += self.__GRASP_OBJ_OFFSET_PITCH
        grasp_ori_rpy = [roll, pitch, yaw]

        if offset:
            # オフセット量の考慮
            grasp_pos = [pos + off for pos, off in zip(grasp_pos, self.__grasp_obj_offset)]

        if dim2:
            grasp_pos = grasp_pos[:DIMENTION_2D]

        return grasp_pos, grasp_ori_rpy


    @property
    def grasp_obj_id(self):
        """
        _grasp_obj_idプロパティのゲッター
        """
        return self.__grasp_obj_id
