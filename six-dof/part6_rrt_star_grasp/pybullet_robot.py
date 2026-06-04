# PyBulletで使用するロボットを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import time             # 時間
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *
from pybullet_gripper import ParallelGripper



class _PyBulletRobot:
    # 定数の定義
    # 子クラスで必ず定義する必要がある ↓
    _DIM_JOINT = DIMENTION_NONE     # 関節の次元数
    _DIM_POSE  = DIMENTION_NONE     # 位置(姿勢も含む)の次元数
    # 子クラスで必ず定義する必要がある ↑
    
    _N_HAND_JOINT = 4               # ハンド用の関節数 (パラレルグリッパーだけに対応するため，4固定)
    _N_HAND_JOINT_NOT_FIXED = 2     # ハンド用の関節で固定関節以外
    
    _WEIGHT_JOINT = None            # 各軸への重み (RRTで使用
    
    _JOINT_LIMIT_LOW_IDX = 8        # 関節限界の下限値インデックス
    _JOINT_LIMIT_UP_IDX  = 9        # 関節限界の上限値インデックス
    
    
    def __init__(self, robot_id, interpolation, hand):
        """
        コンストラクタ

        パラメータ
            robot_id(int): ロボットID (loadURDF()の戻り値)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンド装着の有無 True/False = あり/なし
        """
        # 引数の確認
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            # 異常
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        # プロパティの更新
        self._robot_id = robot_id
        self._interpolation = interpolation

        # urdf よりロボットの関節数を取得 (エンドエフェクタ用のデバッグ関節は不要なため -1)
        self._n_joints = p.getNumJoints(robot_id) - 1

        # ハンド装着有無
        if hand:        # ハンド装着
            if self._n_joints != (self._DIM_JOINT + self._N_HAND_JOINT):
                # 関節数が異常
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

            # 今回は，パラレルグリッパーのみに対応
            self._hand = ParallelGripper(self._robot_id, self._n_joints)

        else:           # ハンドなし
            if self._n_joints != self._DIM_JOINT:
                # 関節数が異常
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

            self._hand = None

        # URDFから関節限界を取得
        self.__joint_limit()


    def __joint_limit(self):
        """
        関節限界
        """
        joint_limit = []

        # 全軸の関節限界
        for joint_index in range(self._DIM_JOINT):
            joint_info = p.getJointInfo(self._robot_id, joint_index)
            lower = joint_info[self._JOINT_LIMIT_LOW_IDX]
            upper = joint_info[self._JOINT_LIMIT_UP_IDX]
            joint_limit.append((lower, upper))

        # プロパティの更新
        self._joints_limit = np.array(joint_limit)


    @property
    def robot_id(self):
        """
        _robot_idプロパティのゲッター
        """
        return self._robot_id

    @property
    def joints_limit(self):
        """
        _joints_limitプロパティのゲッター
            最小関節限界：[:, 0]
            最大関節限界：[:, 1]
        """
        return self._joints_limit

    @property
    def weight_joint(self):
        """
        _WEIGHT_JOINT (各関節の重み) のゲッター
        """
        return self._WEIGHT_JOINT

    @property
    def interpolation(self):
        """
        _interpolationプロパティのゲッター
        """
        return self._interpolation


    def _chk_pos_dim(self, pos):
        """
        位置(姿勢も含む)の次元数確認

        パラメータ
            pos(numpy.ndarray): 位置 [m]，姿勢 [rad]
        """
        # 引数の確認
        if len(pos) != self._DIM_POSE:
            # 異常
            raise ValueError(f"pos's shape is abnormal. pos'size is {len(pos)}")

    def _chk_thetas_dim(self, thetas):
        """
        関節の次元数確認

        パラメータ
            thetas(numpy.ndarray): 関節角度 [rad]
        """
        if len(thetas) != self._DIM_JOINT:
            # 異常
            raise ValueError(f"theta's shape is abnormal. thetas'size is {len(thetas)}")

    def set_joint(self, thetas):
        """
        関節角度の設定

        パラメータ
            thetas(numpy.ndarray): 関節角度 [rad]
        """
        # 引数の確認
        self._chk_thetas_dim(thetas)

        for i in range(len(thetas)):
            # 関節角度を設定
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id,    # IDの設定
                jointIndex=i,                   # 関節番号の設定
                controlMode=p.POSITION_CONTROL, # 位置制御
                targetPosition=thetas[i]        # 関節角度
            )

    def set_jump_joint(self, thetas):
        """
        関節角度をジャンプ

        パラメータ
            thetas(numpy.ndarray): 関節角度 [rad]
        """
        # 引数の確認
        self._chk_thetas_dim(thetas)

        for i in range(thetas.shape[0]):
            # 関節角度を設定
            p.resetJointState(
                bodyUniqueId=self._robot_id,    # IDの設定
                jointIndex=i,                   # 関節番号の設定
                targetValue=thetas[i]           # 関節角度
            )

    def convert_pos_to_theta(self, pos, force=False):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度
            force(bool): パラメータposを絶対に位置とみなす

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        raise InterruptedError("convert_pos_to_theta() is necessary override.")

    def run_gripper(self, open=False, close=False):
        """
        グリッパーの実行
        
        パラメータ
            open(bool): グリッパーのオープンフラグ
            close(bool): グリッパーのクローズフラグ
        """
        if self._hand is None:
            # ハンド非装着のため，処理終了
            return

        self._hand.run(open, close)


class _PyBullet2DoFRobot(_PyBulletRobot):
    # 定数の定義
    _Z_VALUE = 0.0      # 位置を3次元変換する時のZ値
    
    _DIM_JOINT = DIMENTION_2D     # 関節の次元数
    _DIM_POSE  = DIMENTION_2D     # 位置(姿勢も含む)の次元数
    
    
    def __init__(self, robot_id, interpolation, hand):
        """
        コンストラクタ

        パラメータ
            robot_id(int): ロボットID (loadURDF()の戻り値)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンド装着の有無 True/False = あり/なし
        """
        # 親クラスのコンストラクを実行
        super().__init__(robot_id, interpolation, hand)


    def __inverse_kinematics(self, pos):
        """
        逆運動学(位置から関節角度に変換)

        パラメータ
            pos(numpy.ndarray): 位置 [m]

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        # 引数の確認
        self._chk_pos_dim(pos)

        # posは2次元データであるため，3次元データへ変換する
        # (PyBulletの逆運動学を実装するため)
        pos = np.append(pos, self._Z_VALUE)

        # エンドエフェクタのリンク要素はベースリンクを除いた要素番号となる
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        thetas = np.array(thetas)

        return thetas


    def convert_pos_to_theta(self, pos, force=False):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 [m] / 関節角度 [rad]
            force(bool): パラメータposを絶対に位置とみなす

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        if force:       # posを位置とみなす
            # 逆運動学
            thetas = self.__inverse_kinematics(pos)

        else:           # プロパティ "_interpolation" より決定
            if self._interpolation == INTERPOLATION.POSITION.value:
                # 逆運動学
                thetas = self.__inverse_kinematics(pos)

            else:
                # pos が関節角度のため，そのまま返す
                thetas = np.copy(pos)

        if self._hand is not None:
            # グリッパー付きの場合は，グリッパー部分を削除
            thetas = thetas[:self._DIM_JOINT]

        return thetas


class _PyBullet3DoFRobot(_PyBulletRobot):
    # 定数の定義
    _DIM_JOINT = DIMENTION_3D     # 関節の次元数
    _DIM_POSE  = DIMENTION_3D     # 位置(姿勢も含む)の次元数
    
    
    def __init__(self, robot_id, interpolation, hand):
        """
        コンストラクタ

        パラメータ
            robot_id(int): ロボットID (loadURDF()の戻り値)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンド装着の有無 True/False = あり/なし
        """
        # 親クラスのコンストラクを実行
        super().__init__(robot_id, interpolation, hand)


    def __inverse_kinematics(self, pos):
        """
        逆運動学(位置から関節角度に変換)

        パラメータ
            pos(numpy.ndarray): 位置 [m]

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        # 引数の確認
        self._chk_pos_dim(pos)

        # エンドエフェクタのリンク要素はベースリンクを除いた要素番号となる
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        thetas = np.array(thetas)

        return thetas


    def convert_pos_to_theta(self, pos, force=False):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 [m] / 関節角度 [rad]
            force(bool): パラメータposを絶対に位置とみなす

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        if force:       # posを位置とみなす
            # 逆運動学
            thetas = self.__inverse_kinematics(pos)

        else:           # プロパティ "_interpolation" より決定
            if self._interpolation == INTERPOLATION.POSITION.value:
                # 逆運動学
                thetas = self.__inverse_kinematics(pos)

            else:
                # pos が関節角度のため，そのまま返す
                thetas = np.copy(pos)

        if self._hand is not None:
            # グリッパー付きの場合は，グリッパー部分を削除
            thetas = thetas[:self._DIM_JOINT]

        return thetas


class _PyBullet6DoFRobot(_PyBulletRobot):
    # 定数の定義
    _DIM_JOINT = DIMENTION_6D     # 関節の次元数
    _DIM_POSE  = DIMENTION_6D     # 位置(姿勢も含む)の次元数
    
    # _WEIGHT_JOINT = np.array([1, 1, 1, 1, 1, 1])  # 各軸への重み (RRTで使用)
    
    
    def __init__(self, robot_id, interpolation, hand):
        """
        コンストラクタ

        パラメータ
            robot_id(int): ロボットID (loadURDF()の戻り値)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンド装着の有無 True/False = あり/なし
        """
        # 親クラスのコンストラクを実行
        super().__init__(robot_id, interpolation, hand)


    def __inverse_kinematics(self, pos):
        """
        逆運動学(位置から関節角度に変換)

        パラメータ
            pos(numpy.ndarray): 位置[m]・姿勢[rad]

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        # 引数の確認
        self._chk_pos_dim(pos)

        # パラメータ pos を位置と姿勢に分解
        position, orientation = pos[:DIMENTION_3D], pos[DIMENTION_3D:]
        # 姿勢を ロール・ピッチ・ヨー から クォータニオンへ変換
        # 逆運動学の解を算出するには，姿勢はクォータニオンでないといけない
        quaternion = p.getQuaternionFromEuler(orientation)

        # エンドエフェクタのリンク要素はベースリンクを除いた要素番号となる
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, targetPosition=position, targetOrientation=quaternion)
        thetas = np.array(thetas)

        if self._hand is not None:
            # グリッパー付きなら，グリッパー部分の角度は不要
            thetas = thetas[:self._DIM_JOINT]

        return thetas


    def convert_pos_to_theta(self, pos, force=False):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 [m] / 関節角度 [rad]
            force(bool): パラメータposを絶対に位置とみなす

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        if force:       # posを位置とみなす
            # 逆運動学
            thetas = self.__inverse_kinematics(pos)

        else:           # プロパティ "_interpolation" より決定
            if self._interpolation == INTERPOLATION.POSITION.value:
                # 逆運動学
                thetas = self.__inverse_kinematics(pos)

            else:
                # pos が関節角度のため，そのまま返す
                thetas = np.copy(pos)

                if self._hand is not None:
                # グリッパー付きなら，グリッパー部分の角度は不要
                    thetas = thetas[:self._DIM_JOINT]

        return thetas


class PyBulletRobotController:
    # 定数の定義
    __ROBOT_BASE_POSITION = [0, 0, 0]   # ロボットーのベース位置
    
    
    def __init__(self, n_robot_joint, interpolation, hand):
        """
        コンストラクタ

        パラメータ
            n_robot_joint(int): ロボットアームの関節数(2, 3, 6だけ)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンドの装着有無 True/False = 装着/未装着
        """
        # ロボットアームに応じて，URDFを変える
        if n_robot_joint == DIMENTION_2D:
            # 2軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF2_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF2.value

            # 2軸ロボットアームのクラス
            robot_cls = _PyBullet2DoFRobot

        elif n_robot_joint == DIMENTION_3D:
            # 3軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF3_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF3.value

            # 3軸ロボットアームのクラス
            robot_cls = _PyBullet3DoFRobot

        elif n_robot_joint == DIMENTION_6D:
            # 6軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF6_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF6.value

            # 6軸ロボットアームのクラス
            robot_cls = _PyBullet6DoFRobot

        else:   # 異常
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}.")

        # ロボットを読み込む．ベースリンクの原点は (x, y, z) = (0, 0, 0) として，ベースリンクは地面に固定
        robot_id = p.loadURDF(robot_urdf, basePosition=self.__ROBOT_BASE_POSITION, useFixedBase=True)

        # ロボットクラスのインスタンス作成
        self.__robot = robot_cls(robot_id, interpolation, hand)

        # プロパティの更新
        self.__n_robot_joint = n_robot_joint


    @property
    def robot_id(self):
        """
        PyBulletで割り当てられたロボットID
        """
        return self.__robot.robot_id

    @property
    def joints_limit(self):
        """
        関節限界
            最小関節限界：[:, 0]
            最大関節限界：[:, 1]
        """
        return self.__robot.joints_limit

    @property
    def n_robot_joint(self):
        """
        __n_robot_jointプロパティのゲッター
        """
        return self.__n_robot_joint

    @property
    def weight_joint(self):
        """
        各関節の重みを取得
        """
        return self.__robot.weight_joint

    @property
    def interpolation(self):
        """
        __interpolationプロパティのゲッター
        """
        return self.__robot.interpolation


    def set_joint(self, thetas):
        """
        関節角度の設定

        パラメータ
            thetas(numpy.ndarray): 関節角度 [rad]
        """
        self.__robot.set_joint(thetas)

    def set_jump_joint(self, thetas):
        """
        関節角度をジャンプ

        パラメータ
            thetas(numpy.ndarray): 関節角度 [rad]
        """
        self.__robot.set_jump_joint(thetas)

    def convert_pos_to_theta(self, pos, force=False):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度
            force(bool): パラメータposを絶対に位置とみなす

        戻り値
            numpy.ndarray: 関節角度 [rad]
        """
        return self.__robot.convert_pos_to_theta(pos, force)

    def run_gripper(self, open=False, close=False):
        """
        グリッパーの実行
        
        パラメータ
            open(bool): グリッパーのオープンフラグ
            close(bool): グリッパーのクローズフラグ
        """
        self.__robot.run_gripper(open=open, close=close)
