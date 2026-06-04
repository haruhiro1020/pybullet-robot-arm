# PyBulletで使用するロボットを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import pybullet_data    # PyBulletで使用するデータ
import time             # 時間
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *
from pybullet_rrt import RRTPyBullet



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
    _GRIPPER_RIGHT_IDX =  -2    # 右グリッパーの関節番号
    _GRIPPER_LEFT_IDX  =  -1    # 左グリッパーの関節番号
    
    _GRIPPER_MOVE_VAL  = 0.01   # グリッパーの1回あたりの移動量 [m]
    _GRIPPER_LATERAL_FRIC = 1.0 # グリッパーの摩擦係数
    
    _GRIPPER_CLOSE_VAL = 0.03   # グリッパーのクローズ時のフィンガー角度 [m]
    _GRIPPER_OPEN_VAL  = 0      # グリッパーのクローズ時のフィンガー角度 [m]
    
    
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
        self._chg_dynamics()
    
    
    def _chg_dynamics(self):
        """
        ダイナミクスの変更
        """
        # グリッパーの右・左の関節番号を取得
        gripper_right_left_idx = self._get_gripper_right_left_idx()

        for idx in gripper_right_left_idx:
            p.changeDynamics(self._robot_id,        # 把持対象物ID
                            idx,                    # 関節番号
                            lateralFriction=1.0)    # 床との摩擦係数
    
    
    def run(self, open=False, close=False):
        """
        実行 (毎時刻，本関数を呼ぶこと)

        パラメータ
            open(bool): グリッパーのオープンフラグ
            close(bool): グリッパーのクローズフラグ
        """
        # グリッパーの現在の関節角度[m]を取得
        joint_values = self._get_joint_values()

        # オープンとクローズが同時実行の時，安全の観点よりクローズよりもオープンを優先
        if open:
            # グリッパーのオープン時
            # 移動方向を取得して，設定したい関節角度[m]を計算
            direction = self._get_move_direction(open=open)
            joint_values = direction * self._GRIPPER_OPEN_VAL
        elif close:
            # グリッパーのクローズ時
            # グリッパーのクローズに関するキーボードが押下された時
            direction = self._get_move_direction(open=open)
            joint_values = direction * self._GRIPPER_CLOSE_VAL
        else:
            # 押下されていないため，何もしない
            pass

        # 関節角度の設定
        self._set_joint_values(joint_values)

    def _get_move_direction(self, open):
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

    def _get_gripper_right_left_idx(self):
        """
        グリッパーの右・左の関節番号

        戻り値
            list: グリッパーの右・左の関節番号 (右・左の順番にデータ保存)
        """
        right_left_idx = [self._n_joint + self._GRIPPER_RIGHT_IDX, self._n_joint + self._GRIPPER_LEFT_IDX]

        return right_left_idx

    def _set_joint_values(self, values):
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
        gripper_right_left_idx = self._get_gripper_right_left_idx()

        for idx, gripper_idx in enumerate(gripper_right_left_idx):
            p.setJointMotorControl2(
                bodyIndex=self._robot_id,
                jointIndex=gripper_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=values[idx],
                positionGain=0.5,      # デフォルトよりやや高め
                velocityGain=1.0       # 高速応答（必要に応じて調整）
            )

    def _get_joint_values(self):
        """
        関節角度[m]を取得

        戻り値
            numpy.ndarray: 関節角度 (グリッパーの右関節，グリッパーの左関節)
        """
        # グリッパーの右・左の関節番号
        gripper_right_left_idx = self._get_gripper_right_left_idx()
        joint_values = []

        for gripper_idx in gripper_right_left_idx:
            # グリッパー関節の状態を取得
            joint_state = p.getJointState(bodyUniqueId=self._robot_id, jointIndex=gripper_idx)
            # 関節の値を保存
            joint_values.append(joint_state[self._JOINT_CURRENT_VALUE_IDX])

        return np.array(joint_values)


class PyBulletRobot:
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
        self._joint_limit()


    def _joint_limit(self):
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
        self._joint_limit = np.array(joint_limit)


    @property
    def joint_limit(self):
        """
        _joint_limitプロパティのゲッター
            最小関節限界：[:, 0]
            最大関節限界：[:, 1]
        """
        return self._joint_limit

    @property
    def weight_joint(self):
        """
        _WEIGHT_JOINT (各関節の重み) のゲッター
        """
        return self._WEIGHT_JOINT


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


class PyBullet2DoFRobot(PyBulletRobot):
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


class PyBullet3DoFRobot(PyBulletRobot):
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


class PyBullet6DoFRobot(PyBulletRobot):
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


class MainPyBulletRobot:
    # 定数の定義
    _PLANE_URDF     = "plane.urdf"  # 地面に関する urdf ファイル
    
    _IDX_MIN_JOINT  = 8             # 関節の最小値が保存されている要素番号
    _IDX_MAX_JOINT  = 9             # 関節の最大値が保存されている要素番号
    
    _SIMULATION_SLEEP_TIME = 0.05   # シミュレーションの待機時間 [sec]
    
    _INTERFERENCE_MARGIN   = 0.15   # 干渉判定のマージン [m]
    
    _PATH_PLAN_TIME = 100   # 経路生成の最大時間 [sec]
    _N_MARGIN_MOVE  = 50    # 経路生成終了後の余白時間 [回]
    
    _ROBOT_BASE_POSITION      = [ 0,   0,   0]      # ロボットーのベース位置
    # 2軸ロボットアーム用データ
    _GRASP_OBJECT_POS_2DOF    = [ 1.8, 1.0, 0.05]   # 把持対象物の位置
    _GRASP_OBJECT_OFFSET_2DOF = [-0.4, 0]           # 把持対象物の位置のオフセット
    _ENVIRONMENT_POS_2DOF     = [ 0,   0,   0]      # 環境のベース位置
    # 3軸ロボットアーム用データ
    _GRASP_OBJECT_POS_3DOF    = [ 1.25,  0.4, 0.55] # 把持対象物の位置
    _GRASP_OBJECT_OFFSET_3DOF = [-0.15,  0,   0.15] # 把持対象物の位置のオフセット
    _ENVIRONMENT_POS_3DOF     = [ 1.5,  0,   -0.5]  # 環境のベース位置
    # 6軸ロボットアーム用データ
    _GRASP_OBJECT_POS_6DOF    = [ 1.5 ,  0.4,  1.05] # 把持対象物の位置
    _GRASP_OBJECT_OFFSET_6DOF = [ -0.2,  0,    0.2]    # 把持対象物の位置のオフセット
    _ENVIRONMENT_POS_6DOF     = [ 1.75,  0,    0]  # 環境のベース位置
    
    
    def __init__(self, interpolation, n_robot_joint, environment_urdf=None, grasp_urdf=None, hand=False):
        """
        コンストラクタ

        パラメータ
            interpolation(str): 補間方法 (関節空間/位置空間)
            n_robot_joint(int): ロボットアームの関節数(2, 3, 6だけ)
            environment_urdf(str): 環境のファイル名 (urdf)
            grasp_object(str): 把持対象物のファイル名 (urdf)
            hand(bool): ハンドの装着有無 True/False = 装着/未装着
        """
        # PyBulletの初期化
        p.connect(p.GUI)
        # パスの追加
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # シミュレーションの初期化
        p.resetSimulation()
        # 重力の設定 (下(-z軸)方向の加速度)
        p.setGravity(0, 0, -GRABITY_VALUE)

        # ロボットの初期化
        self._init_robot(n_robot_joint, interpolation, hand)

        # 環境の初期化
        self._init_environment(environment_urdf, grasp_urdf)

        self._rrt = None


    def _init_robot(self, n_robot_joint, interpolation, hand):
        """
        ロボットの初期化

        パラメータ
            n_robot_joint(int): ロボットアームの関節数(2, 3, 6だけ)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンドの装着有無 True/False = 装着/未装着
        """
        if n_robot_joint == DIMENTION_2D:       # 2軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF2_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF2.value
            # 2軸ロボットアームのクラス
            robot_cls = PyBullet2DoFRobot

        elif n_robot_joint == DIMENTION_3D:     # 3軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF3_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF3.value
            robot_cls = PyBullet3DoFRobot

        elif n_robot_joint == DIMENTION_6D:     # 6軸ロボットアーム
            if hand:    # ハンド装着
                robot_urdf = ROBOTURDF.DOF6_HAND.value
            else:       # ハンド未装着
                robot_urdf = ROBOTURDF.DOF6.value
            robot_cls = PyBullet6DoFRobot

        else:   # 異常
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}.")

        # ロボットを読み込む．ベースリンクの原点は (x, y, z) = (0, 0, 0) として，ベースリンクは地面に固定
        self._robot_id = p.loadURDF(robot_urdf, basePosition=self._ROBOT_BASE_POSITION, useFixedBase=True)

        # ロボットクラスのインスタンス作成
        self._robot = robot_cls(self._robot_id, interpolation, hand)

        # プロパティの更新
        self._interpolation = interpolation
        self._n_robot_joint = n_robot_joint

    def _init_environment(self, environment_urdf, grasp_urdf):
        """
        環境の初期化
        
        パラメータ
            environment_urdf(str): 環境が保存されているファイル名
            grasp_urdf(str): 把持対象物が保存されているファイル名
        """
        # 地面を読み込む (pybulletが提供している "plane.urdf" を読み込む)
        p.loadURDF(self._PLANE_URDF)

        # 環境を読み込む
        self._environment_id = None
        if environment_urdf is not None:
            if self._n_robot_joint == DIMENTION_2D:     # 2軸ロボットアーム
                basePosition = self._ENVIRONMENT_POS_2DOF
            elif self._n_robot_joint == DIMENTION_3D:       # 3軸ロボットアーム
                basePosition = self._ENVIRONMENT_POS_3DOF
            else:       # 6軸ロボットアーム
                basePosition = self._ENVIRONMENT_POS_6DOF

            self._environment_id = p.loadURDF(environment_urdf, basePosition=basePosition, useFixedBase=True)

        # 把持対象物を読み込む
        self._grasp_id = None
        self._grasp_constraint_id = None

        if grasp_urdf is not None:
            if self._n_robot_joint == DIMENTION_2D:     # 2軸ロボットアーム
                grasp_center = self._GRASP_OBJECT_POS_2DOF
            elif self._n_robot_joint == DIMENTION_3D:   # 3軸ロボットアーム
                grasp_center = self._GRASP_OBJECT_POS_3DOF
            else:       # 6軸ロボットアーム
                grasp_center = self._GRASP_OBJECT_POS_6DOF

            self._grasp_id = p.loadURDF(grasp_urdf, basePosition=grasp_center)

            # 把持対象物に摩擦を付与する
            p.changeDynamics(self._grasp_id,        # 把持対象物ID
                            -1,                     # ベースに対して
                            lateralFriction=1.0,    # 床との摩擦係数
                            spinningFriction=1.0,   # 回転摩擦係数
                            rollingFriction=1)      # 転がり摩擦

            # 把持対象物の位置・姿勢を取得
            grasp_pos, grasp_ori = self._get_grasp_pos(offset=[0, 0, 0])
            # 把持対象物に拘束条件を付与
            self._grasp_constraint_id = self._set_constraint(self._grasp_id, grasp_pos, grasp_ori)


    def run(self, start_pos, path_plan):
        """
        実行
            始点から終点まで，干渉しない経路を生成

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            path_plan(str): 経路生成手法
        
        戻り値
            result(bool): True/False = 経路生成に成功/失敗
        """
        # リアルタイムでのシミュレーション
        p.setRealTimeSimulation(1)

        # 把持対象物の位置を取得
        if self._n_robot_joint == DIMENTION_2D:     # 2軸ロボットアーム
            offset = self._GRASP_OBJECT_OFFSET_2DOF
            end_pos, _ = self._get_grasp_pos(offset, dim2=True)

        elif self._n_robot_joint == DIMENTION_3D:   # 3軸ロボットアーム
            offset = self._GRASP_OBJECT_OFFSET_3DOF
            end_pos, _ = self._get_grasp_pos(offset)

        else:       # 6軸ロボットアーム
            offset = self._GRASP_OBJECT_OFFSET_6DOF
            end_pos, end_ori = self._get_grasp_pos(offset)
            end_pos += end_ori
            # 逆運動学(位置から関節角度へ変換) → 6軸ロボットアームは関節空間だけの対応だから
            end_pos = self._robot.convert_pos_to_theta(end_pos, force=True)

        end_pos = np.array(end_pos)

        # 始点と終点で干渉していないかの確認
        self._is_interference_start_end_pos(start_pos, end_pos)

        # 経路生成手法の設定
        self._set_path_plan(path_plan)

        # 経路生成の実装
        result = self._path_planning(start_pos, end_pos)

        if result:
            # 経路生成に成功
            self._post_path_planning(start_pos, end_pos)

        # ファイル保存
        self._rrt.save()

        # 経路生成後の余白時間
        # self._exec_margin_time(end_pos, np.array(offset))
        self._exec_margin_time()

        return result

    def _exec_margin_time(self):
        """
        経路生成後の余白時間の処理
        """
        # 把持対象物の位置を取得
        end_pos, end_ori = self._get_grasp_pos(offset=[0, 0, 0])
        if self._n_robot_joint == DIMENTION_2D:
            end_pos = np.array(end_pos)[:DIMENTION_2D]
            # 把持対象物への位置へ移動
            end_theta = self._robot.convert_pos_to_theta(end_pos)
        elif self._n_robot_joint == DIMENTION_3D:
            end_pos = np.array(end_pos)
            # 把持対象物への位置へ移動
            end_theta = self._robot.convert_pos_to_theta(end_pos)
        else:
            end_pos = np.array(end_pos + end_ori)
            # 把持対象物への位置へ移動
            end_theta = self._robot.convert_pos_to_theta(end_pos, force=True)

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self._N_MARGIN_MOVE):
            # 終点に移動
            self._robot.set_joint(end_theta)
            # グリッパーの実行
            self._robot.run_gripper(open=True)
            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)

        # 把持物体の拘束を解除
        if self._grasp_constraint_id is not None:
            self._release_constraint(self._grasp_constraint_id)
            time.sleep(self._SIMULATION_SLEEP_TIME)
            self._grasp_constraint_id = None

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self._N_MARGIN_MOVE):
            # 終点に移動
            self._robot.set_joint(end_theta)
            # グリッパーの実行
            self._robot.run_gripper(close=True)
            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _set_constraint(self, object_id, pos, ori):
        """
        拘束条件の設定

        パラメータ
            object_id(int): 拘束したい対象物ID
            pos(list): 拘束したい位置
            ori(list): 拘束したい姿勢

        戻り値
            int: 拘束条件ID
        """
        constraint_id = p.createConstraint(
                            object_id,      # 親番号(拘束したい対象物ID)
                            -1,             # 親リンクの要素番号("-1"はベース)
                            -1,             # 子番号("-1"はなし)
                            -1,             # 子リンクの要素番号("-1"はベース)
                            p.JOINT_FIXED,  # 関節タイプ(今回は固定"JOINT_FIXED")
                            [0, 0, 0],      # 関節軸
                            [0, 0, 0],      # 親の中心からの位置 
                            pos,            # 子の中心からの位置 (今回は子を設定していないから，ワールド座標系から見た関節位置)
                            parentFrameOrientation=ori,         # 親の中心からの姿勢
                            childFrameOrientation=[0, 0, 0, 1]) # 子の中心からの姿勢 (今回は，子を設定していないから，ワールド座標系から見た姿勢)

        return constraint_id

    def _release_constraint(self, constraint_id):
        """
        拘束条件の解除
        """
        if constraint_id is not None:
            # 拘束条件を解除
            p.removeConstraint(constraint_id)

    def _get_grasp_pos(self, offset=None, dim2=False):
        """
        把持対象物の位置を取得

        パラメータ
            offset(list): 把持対象物へのオフセット量 (x, y, z)
            dim2(bool): 2次元位置として取得するかどうか

        戻り値
            list: 把持対象物の位置 [m]
            list: 把持対象物の姿勢 (ロール・ピッチ・ヨー [rad])
        """
        if self._grasp_id is None:  # 把持対象物が存在しない
            raise ValueError("self._grasp_id is abnorma. please set grasp_urdf.")

        # 把持対象物の位置[m]・姿勢[rad]を取得
        grasp_pos, grasp_ori = p.getBasePositionAndOrientation(self._grasp_id)
        # 姿勢をクォータニオンからロール・ピッチ・ヨーへ変換
        roll, pitch, yaw = p.getEulerFromQuaternion(grasp_ori)
        # ピッチ角を90度回転させる → 把持対象物の正面(x方向)から把持したいから
        pitch += np.pi/2
        grasp_ori_rpy = [roll, pitch, yaw]

        if offset is not None:
            # オフセット量の考慮
            grasp_pos_offset = [pos + off for pos, off in zip(grasp_pos, offset)]

        if dim2:
            grasp_pos_offset = grasp_pos_offset[:DIMENTION_2D]

        return grasp_pos_offset, grasp_ori_rpy

    def _set_path_plan(self, path_plan):
        """
        経路生成手法の設定

        パラメータ
            path_plan(str): 経路生成手法名
        """
        if path_plan == PATHPLAN.RRT.value:
            # RRT
            self._rrt = RRTPyBullet()
        else:
            # 異常
            raise ValueError(f"path_plan is abnormal. path_plan is {path_plan}")

    def _path_planning(self, start_pos, end_pos):
        """
        始点から終点までの経路生成

        パラメータ
            start_pos(numpy.ndarray): 始点
            end_pos(numpy.ndarray): 終点

        戻り値
            result(bool): True/False = 経路生成に成功/失敗
        """
        result = False

        # 経路生成の準備
        self._rrt.preparation(start_pos, end_pos, self._interpolation)

        # 関節限界の設定
        self._set_joint_limit()

        start_time = time.time()

        # 始点から終点までの経路が生成するまでループ
        while True:
            now_time = time.time()
            if (now_time - start_time) >= self._PATH_PLAN_TIME:
                # タイムアウト
                break

            # 経路生成を1度実行
            new_node_pos, near_node_pos, near_node = self._rrt.expand_once(end_pos, self._robot.weight_joint)
            if self._is_line_interference(new_node_pos, near_node_pos):
                # 干渉あり
                continue

            # 干渉なしだから，ノード追加 + 経路生成の完了確認
            if not self._rrt.add_node_and_chk_goal(end_pos, new_node_pos, near_node):
                # 終点までの近傍ではない
                continue

            # 新規ノードと始点までの干渉確認
            if not self._is_line_interference(new_node_pos, end_pos):
                # 始点から終点までの経路生成に成功
                result = True
                break

        return result

    def _set_joint_limit(self):
        """
        関節限界を設定
        """
        if self._interpolation == INTERPOLATION.JOINT.value:    # 探索空間が関節空間
            # RRTの探索範囲をロボットの関節限界とする
            self._rrt.set_strict_pos(self._robot.joint_limit[:, 0], self._robot.joint_limit[:, 1])

    def _post_path_planning(self, start_pos, end_pos):
        """
        経路生成の後処理 (経路生成成功時だけ実装)

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
        """
        # 経路生成の終了処理
        self._rrt.fin_planning(start_pos, end_pos)

        # 始点に移動
        theta = self._robot.convert_pos_to_theta(start_pos)
        self._robot.set_jump_joint(theta)
        # グリッパーの実行
        self._robot.run_gripper(open=True)

        # 始点から終点までの経路を移動
        for row_idx in range(self._rrt.pathes.shape[0]):
            next_theta = self._robot.convert_pos_to_theta(self._rrt.pathes[row_idx])
            self._robot.set_joint(next_theta)
            # グリッパーの実行
            self._robot.run_gripper(open=True)
            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)


    def _is_line_interference(self, pos1, pos2):
        """
        2点間の干渉判定

        パラメータ
            pos1(numpy.ndarray): 位置1
            pos2(numpy.ndarray): 位置2

        戻り値
            is_interference(bool): True/False = 干渉あり/干渉なし
        """
        is_interference = True

        # 2点の干渉判定
        if self._is_interference_pos(pos2):
            return is_interference
        if self._is_interference_pos(pos1):
            return is_interference

        # pos1からpos2へ移動
        theta = self._robot.convert_pos_to_theta(pos2)
        self._robot.set_joint(theta)
        # グリッパーの実行
        self._robot.run_gripper(open=True)

        # 待機時間
        time.sleep(self._SIMULATION_SLEEP_TIME)

        # ロボットと干渉物との干渉判定
        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:  # 干渉なし
            is_interference = False

        return is_interference

    def _is_interference_start_end_pos(self, start_pos, end_pos):
        """
        始点と終点が干渉判定していないかの確認

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点 (直交空間/関節空間)
            end_pos(numpy.ndarray): 経路生成の終点 (直交空間/関節空間)
        """
        # 始点の干渉判定
        if self._is_interference_pos(start_pos):
            raise ValueError("start_pos is interference. change start_pos.")

        # 終点の干渉判定
        if self._is_interference_pos(end_pos):
            raise ValueError("end_pos is interference. change end_pos.")

    def _is_interference_pos(self, pos):
        """
        位置にジャンプして干渉判定

        パラメータ
            pos(numpy.ndarray): 位置/関節
        
        戻り値
            is_interference(bool): True/False = 干渉あり/干渉なし
        """
        is_interference = True

        # 位置から関節角度に変換
        theta = self._robot.convert_pos_to_theta(pos)
        # 位置にジャンプ
        self._robot.set_jump_joint(theta)
        # グリッパーの実行
        self._robot.run_gripper(open=True)

        # 待機時間
        time.sleep(self._SIMULATION_SLEEP_TIME)

        # ロボットと干渉物との干渉判定
        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:  # 干渉なし
            is_interference = False

        return is_interference


    def convert_pos_to_theta(self, pos):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度

        戻り値
            thetas(numpy.ndarray): 関節角度
        """
        thetas = self._robot.convert_pos_to_theta(pos, force=True)

        return thetas
