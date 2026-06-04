# PyBulletで使用するグリッパーを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *



class BaseGripper:
    # 定数の定義
    _JOINT_CURRENT_VALUE_IDX = 0    # 関節の現在値の要素番号
    
    
    def __init__(self, robot_id, n_joint):
        """
        コンストラクタ

        パラメータ
            robot_id(p.loadURDFの戻り値): ロボットURDFを読み込んだ際のID
            n_joint(int): ロボットの関節数 (グリッパーは含むが，グリッパー先端は含まない)
        """
        # プロパティの初期化
        self._robot_id = robot_id
        self._n_joint  = n_joint
        
        
    def run(self):
        """
        実行 (毎時刻，本関数を呼ぶこと)
        """
        raise InterruptedError("run() is necessary override.")


class ParallelGripper(BaseGripper):
    # 定数の定義
    __GRIPPER_RIGHT_IDX =  -2   # 右グリッパーの関節番号
    __GRIPPER_LEFT_IDX  =  -1   # 左グリッパーの関節番号
    
    __LATERAL_FRICTION  = 1.0   # 摩擦係数
    
    __CLOSE_VAL = 0.03          # クローズ時のフィンガー角度 [m]
    __OPEN_VAL  = 0             # オープン時のフィンガー角度 [m]
    
    
    def __init__(self, robot_id, n_joint):
        """
        コンストラクタ

        パラメータ
            robot_id(p.loadURDFの戻り値): ロボットURDFを読み込んだ際のID
            n_joint(int): ロボットの関節数 (グリッパーは含むが，グリッパー先端は含まない)
        """
        # 親クラスのコンストラクタ
        super().__init__(robot_id, n_joint)

        # ダイナミクスの変更
        self.__chg_dynamics()
    
    
    def __chg_dynamics(self):
        """
        ダイナミクスの変更
        """
        # グリッパーの右・左の関節番号を取得
        gripper_right_left_idx = self.__get_gripper_right_left_idx()

        for idx in gripper_right_left_idx:
            p.changeDynamics(self._robot_id,        # 把持対象物ID
                            idx,                    # 関節番号
                            lateralFriction=self.__LATERAL_FRICTION)    # 床との摩擦係数
    
    
    def run(self, open=False, close=False):
        """
        実行 (毎時刻，本関数を呼ぶこと)

        パラメータ
            open(bool): グリッパーのオープンフラグ
            close(bool): グリッパーのクローズフラグ
        """
        # グリッパーの現在の関節角度[m]を取得
        joint_values = self.__get_joint_values()

        # オープンとクローズが同時実行の時，安全の観点よりクローズよりもオープンを優先
        if open:
            # グリッパーのオープン時
            # 移動方向を取得して，設定したい関節角度[m]を計算
            direction = self.__get_move_direction(open=open)
            joint_values = direction * self.__OPEN_VAL
        elif close:
            # グリッパーのクローズ時
            # グリッパーのクローズに関するキーボードが押下された時
            direction = self.__get_move_direction(open=open)
            joint_values = direction * self.__CLOSE_VAL
        else:
            # 押下されていないため，何もしない
            pass

        # 関節角度の設定
        self.__set_joint_values(joint_values)

    def __get_move_direction(self, open):
        """
        グリッパーの移動方向を取得

        パラメータ
            open(bool): True/False = オープン/クローズ

        戻り値
            numpy.ndarray: グリッパーの移動方向 (右関節・左関節の順番)
        """
        # グリッパーの右関節・左関節の移動方向
        move_direction = np.array([-1.0, 1.0])
        if not open:        # クローズ
            move_direction *= -1

        return move_direction

    def __get_gripper_right_left_idx(self):
        """
        グリッパーの右・左の関節番号

        戻り値
            list: グリッパーの右・左の関節番号 (右・左の順番にデータ保存)
        """
        right_left_idx = [self._n_joint + self.__GRIPPER_RIGHT_IDX, self._n_joint + self.__GRIPPER_LEFT_IDX]

        return right_left_idx

    def __set_joint_values(self, values):
        """
        関節角度[m]を設定

        パラメータ
            values(numpy.ndarray): 設定値
        """
        # パラメータのサイズを確認
        if values.shape[0] != DIMENTION_2D:
            # 異常
            raise ValueError(f"values'shape[0] is abnormal. values'shape[0] is {values.shape[0]}")

        # グリッパーの右・左の関節番号
        gripper_right_left_idx = self.__get_gripper_right_left_idx()

        for idx, gripper_idx in enumerate(gripper_right_left_idx):
            p.setJointMotorControl2(
                bodyIndex=self._robot_id,
                jointIndex=gripper_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=values[idx],
                positionGain=0.5,      # デフォルトよりやや高め
                velocityGain=1.0       # 高速応答（必要に応じて調整）
            )

    def __get_joint_values(self):
        """
        関節角度[m]を取得

        戻り値
            numpy.ndarray: 関節角度 (グリッパーの右関節，グリッパーの左関節)
        """
        # グリッパーの右・左の関節番号
        gripper_right_left_idx = self.__get_gripper_right_left_idx()
        joint_values = []

        for gripper_idx in gripper_right_left_idx:
            # グリッパー関節の状態を取得
            joint_state = p.getJointState(bodyUniqueId=self._robot_id, jointIndex=gripper_idx)
            # 関節の値を保存
            joint_values.append(joint_state[self._JOINT_CURRENT_VALUE_IDX])

        return np.array(joint_values)
