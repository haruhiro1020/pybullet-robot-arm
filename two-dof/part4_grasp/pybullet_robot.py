# PyBulletで使用するロボットを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import pybullet_data    # PyBulletで使用するデータ
import time             # 時間
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
    _GRIPPER_RIGHT_IDX =  -2    # 右グリッパーの関節番号
    _GRIPPER_LEFT_IDX  =  -1    # 左グリッパーの関節番号
    
    _GRIPPER_MOVE_VAL  = 0.01   # グリッパーの1回あたりの移動量 [m]
    _GRIPPER_LATERAL_FRIC = 1.0 # グリッパーの摩擦係数
    
    
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
    
    
    def run(self):
        """
        実行 (毎時刻，本関数を呼ぶこと)
        """
        # キーボードの押下状況を取得
        keys = p.getKeyboardEvents()

        # グリッパーの現在の関節角度[m]を取得
        joint_values = self._get_joint_values()

        # オープンとクローズが同時実行の時，安全の観点よりクローズよりもオープンを優先
        if self._chk_down_key(KEYBOARD.GRIP_OPEN.value, keys):
            # グリッパーのオープンに関するキーボードが押下された時
            # 移動方向を取得して，設定したい関節角度[m]を計算
            direction = self._get_move_direction(open=True)
            joint_values += direction * self._GRIPPER_MOVE_VAL
        elif self._chk_down_key(KEYBOARD.GRIP_CLOSE.value, keys):
            # グリッパーのクローズに関するキーボードが押下された時
            direction = self._get_move_direction(open=False)
            joint_values += direction * self._GRIPPER_MOVE_VAL
        else:
            # 押下されていないため，何もしない
            pass

        # 関節角度の設定
        self._set_joint_values(joint_values)

    def _chk_down_key(self, key, keys):
        """
        特定のキーが押下されているかの確認

        パラメータ
            key(int): 特定のキー
            keys(list): キーボード情報

        戻り値
            bool: True/False = 押下されている/されていない
        """
        is_down = False

        # 押下確認
        if key in keys and keys[key] & p.KEY_IS_DOWN:
            is_down = True

        return is_down

    def _get_move_direction(self, open):
        """
        グリッパーの移動方向を取得

        パラメータ
            open(bool): True/False = オープン/クローズ

        戻り値
            numpy.ndarray: グリッパーの移動方向 (右関節・左関節の順番)
        """
        # グリッパーの右関節・左関節の移動方向
        move_direction = np.array([1.0, -1.0])
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


class MainPyBulletRobot:
    # 定数の定義
    _PLANE_URDF     = "plane.urdf"  # 地面に関する urdf ファイル
    
    _IDX_MIN_JOINT  = 8             # 関節の最小値が保存されている要素番号
    _IDX_MAX_JOINT  = 9             # 関節の最大値が保存されている要素番号
    
    _SLIDER_MAKE_WAIT_TIME = 0.2    # スライダー作成の待機時間 [sec]
    _SIMULATION_SLEEP_TIME = 1. / 240.   # シミュレーションの待機時間 [sec]
    
    _DEBUG_TEXT_LIFE_TIME  = 0      # テキストの生存時間 [sec] (0は無限時間)
    _DEBUG_TEXT_SIZE       = 0.5    # テキストの大きさ
    
    _ZERO_NEAR = 1e-4               # 0近傍の値
    
    _INTERFERENCE_MARGIN   = 0.1    # 干渉判定のマージン [m]
    
    _PATH_PLAN_TIME = 1000      # 経路生成の最大時間 [sec]
    _N_MARGIN_MOVE  = 100       # 経路生成終了後の余白時間 [回]
    
    _N_HAND_JOINT   = 4         # ハンド用の関節数
    
    _KEY_DOWN_MOVE_POS = 0.1   # キーボード押下時のロボット手先位置の移動量 [m]
    _KEY_DOWN_MOVE_ORI = 0.05   # キーボード押下時のロボット手先姿勢の移動量 [rad]
    
    
    def __init__(self, interpolation, robot_urdf, environment_urdf=None, grasp_urdf=None, hand=False):
        """
        コンストラクタ

        パラメータ
            interpolation(str): 補間方法 (関節空間/位置空間)
            robot_urdf(str): ロボットアームのファイル名 (urdf)
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
        self._init_robot(robot_urdf, interpolation, hand)

        # 環境の初期化
        self._init_environment(environment_urdf, grasp_urdf)


    def _init_robot(self, robot_urdf, interpolation, hand):
        """
        ロボットの初期化

        パラメータ
            robot_urdf(str): ロボットアームのファイル名 (urdf)
            interpolation(str): 探索方法 (関節空間/位置空間)
            hand(bool): ハンドの装着有無 True/False = 装着/未装着
        """
        # 引数の確認
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            # 異常
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        # プロパティの更新
        self._interpolation = interpolation

        # ロボットを読み込む．ベースリンクの原点は (x, y, z) = (0, 0, 0) として，ベースリンクは地面に固定
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # urdf よりロボットの関節数を取得 (エンドエフェクタ用のデバッグ関節は不要なため -1)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        # ハンド装着有無
        if hand:
            # ハンド装着
            if self._n_joints != (DIMENTION_2D + self._N_HAND_JOINT):
                # 2軸ロボットアーム以外
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

            self._hand = ParallelGripper(self._robot_id, self._n_joints)

        else:
            # ハンド非装着
            if self._n_joints != DIMENTION_2D:
                # 2軸ロボットアーム以外
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

            self._hand = None

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
            self._environment_id = p.loadURDF(environment_urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # 把持対象物を読み込む
        self._grasp_id = None
        if grasp_urdf is not None:
            self._grasp_id = p.loadURDF(grasp_urdf, basePosition=[0, 0, 0.1])

            # 把持対象物に摩擦を付与する
            p.changeDynamics(self._grasp_id,        # 把持対象物ID
                            -1,                     # ベースに対して
                            lateralFriction=1.0,    # 床との摩擦係数
                            spinningFriction=1.0,   # 回転摩擦係数
                            rollingFriction=1)      # 転がり摩擦


    def run(self):
        """
        実行
            始点から終点まで，干渉しない経路を生成
        """
        # リアルタイムでのシミュレーション
        p.setRealTimeSimulation(1)

        while True:
            # 新しい位置を取得
            new_pos = self._calc_new_pos()

            # 回転角度に変換 (ロボットへ渡せるのは関節角度だけだから)
            thetas = self._convert_pos_to_theta(new_pos)
            # 関節角度を設定
            self._set_joint(thetas)

            # グリッパーの実行
            self._run_gripper()

    def _calc_new_pos(self):
        """
        新しい位置を計算

        戻り値
            numpy.ndarray: 新しい位置
        """
        # ロボットの手先位置を取得
        ee_pos = np.array(self._get_ee_state()[0])
        # キーボードの値を取得
        keys = p.getKeyboardEvents()

        # x方向への移動量を取得
        x_value = self._get_move_pos(KEYBOARD.PLUS_X.value, KEYBOARD.MINUS_X.value, keys)
        ee_pos[0] += x_value
        # y方向への移動量を取得
        y_value = self._get_move_pos(KEYBOARD.PLUS_Y.value, KEYBOARD.MINUS_Y.value, keys)
        ee_pos[1] += y_value

        return ee_pos

    def _get_move_pos(self, plus_key, minus_key, keys):
        """
        位置の移動量を取得

        パラメータ
            plus_key(int): +方向を割り当てたキーボード
            minus_key(int): -方向を割り当てたキーボード
            keys(list): キーボード情報

        戻り値
            float: 特定方向への移動量
        """
        value = 0.0

        if self._chk_down_key(plus_key, keys):
            # +x方向へ移動
            value += self._KEY_DOWN_MOVE_POS

        if self._chk_down_key(minus_key, keys):
            # -x方向へ移動
            value -= self._KEY_DOWN_MOVE_POS

        return value

    def _chk_down_key(self, key, keys):
        """
        特定のキーが押下されているかの確認

        パラメータ
            key(int): 特定のキー
            keys(list): キーボード情報

        戻り値
            bool: True/False = 押下されている/されていない
        """
        is_down = False

        # 押下確認
        if key in keys and keys[key] & p.KEY_IS_DOWN:
            is_down = True

        return is_down

    def _get_ee_state(self):
        """
        エンドエフェクタの状態を取得

        戻り値
            list: エンドエフェクタの状態
                要素0: ワールド座標系から見た，エンドエフェクタの重心位置
                要素1: ワールド座標系から見た，エンドエフェクタの重心姿勢
                要素2: エンドエフェクタの座標系から見た，エンドエフェクタの重心位置
                要素3: エンドエフェクタの座標系から見た，エンドエフェクタの重心姿勢
                要素4: ワールド座標系から見た，エンドエフェクタの座標系位置
                要素5: ワールド座標系から見た，エンドエフェクタの座標系姿勢
                要素6: ワールド座標系から見た，エンドエフェクタの速度
                要素7: ワールド座標系から見た，エンドエフェクタの角速度
        """
        # エンドエフェクタの位置にテキストを出力したい
        ee_state = p.getLinkState(self._robot_id, self._n_joints)
        return ee_state

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

    def _set_joint(self, thetas):
        """
        関節角度の設定 (現在位置から移動)

        パラメータ
            thetas(numpy.ndarray): 設定したい関節角度
        """
        for i in range(len(thetas)):
            # 関節角度を設定
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id,    # IDの設定
                jointIndex=i,                   # 関節番号の設定
                controlMode=p.POSITION_CONTROL, # 位置制御
                targetPosition=thetas[i]        # 関節角度
            )

    def _run_gripper(self):
        """
        グリッパーの実行
        """
        if self._hand is None:
            # ハンド非装着のため，処理終了
            return

        self._hand.run()


    def convert_pos_to_theta(self, pos):
        """
        位置から関節角度に変換 (クラス外で使う用)

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度

        戻り値
            thetas(numpy.ndarray): 関節角度
        """
        if pos.shape[0] == DIMENTION_2D:
            pos = np.append(pos, 0.0)
        # エンドエフェクタのリンク要素はベースリンクを除いた要素番号となる
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        thetas = np.array(thetas)

        return thetas

    def _convert_pos_to_theta(self, pos):
        """
        位置から関節角度に変換 (クラス内で使う用)

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度

        戻り値
            thetas(numpy.ndarray): 関節角度
        """
        if self._interpolation == INTERPOLATION.POSITION.value:
            # 逆運動学により，関節角度を返す
            # 逆運動学には，(x, y, z)の3次元データが必須なため，2次元の場合はzのデータも増やす
            if pos.shape[0] == DIMENTION_2D:
                pos = np.append(pos, 0.0)
            # エンドエフェクタのリンク要素はベースリンクを除いた要素番号となる
            thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
            thetas = np.array(thetas)
        else:
            # pos が関節角度のため，そのまま返す
            thetas = np.copy(pos)

        return thetas
