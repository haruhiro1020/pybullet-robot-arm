# PyBulletで使用するロボットを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import pybullet_data    # PyBulletで使用するデータ
import time             # 時間
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *


class MainPyBulletRobot:
    # 定数の定義
    _PLANE_URDF     = "plane.urdf"  # 地面に関する urdf ファイル
    
    _IDX_MIN_JOINT  = 8             # 関節の最小値が保存されている要素番号
    _IDX_MAX_JOINT  = 9             # 関節の最大値が保存されている要素番号
    _JOINT_INIT     = 0.0           # 関節の初期値
    
    _SLIDER_MAKE_WAIT_TIME = 0.2    # スライダー作成の待機時間 [sec]
    _SIMULATION_SLEEP_TIME = 1. / 240.    # シミュレーションの待機時間 [sec]
    
    _DEBUG_TEXT_LIFE_TIME  = 0      # テキストの生存時間 [sec] (0は無限時間)
    _DEBUG_TEXT_SIZE       = 0.5    # テキストの大きさ
    
    
    def __init__(self, robot_urdf, interpolation):
        """
        コンストラクタ

        パラメータ
            robot_urdf(str): ロボットアームのファイル名 (urdf)
            interpolation(str): 補間方法 (関節空間/位置空間)
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
        self._init_robot(robot_urdf)

        # 環境の初期化
        self._init_environment()

        # スライダーの初期化
        self._init_sliders(interpolation)

        # スライダー作成に時間がかかるため，少しまつ
        time.sleep(self._SLIDER_MAKE_WAIT_TIME)


    def _init_robot(self, robot_urdf):
        """
        ロボットの初期化

        パラメータ
            robot_urdf(str): ロボットアームのファイル名 (urdf)
        """
        # ロボットを読み込む．ベースリンクの原点は (x, y, z) = (0, 0, 0) として，ベースリンクは地面に固定
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # urdf よりロボットの関節数を取得 (エンドエフェクタ用の仮想関節は不要なため -1)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        # ロボットの関節数に応じて，robot.py内のクラスを決定
        if self._n_joints != DIMENTION_2D:
            # 2軸ロボットアーム以外
            raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

    def _init_environment(self):
        """
        環境の初期化
        """
        # 地面を読み込む (pybulletが提供している "plane.urdf" を読み込む)
        p.loadURDF(self._PLANE_URDF)

    def _init_sliders(self, interpolation):
        """
        スライダーの初期化

        パラメータ
            interpolation(str): 補間方法 (関節空間/位置空間)
        """
        sliders = []

        # GUIにスライダーを追加
        if interpolation == INTERPOLATION.JOINT.value:  # 関節空間を探索
            # 設定できる関節の最小値と最大値を取得
            min_joints, max_joints = self._get_joint_limit()
            # 初期角度を取得
            init_thetas = self._get_init_thetas()

            # 設定できる関節の最小値・最大値・初期値を設定
            for joint_idx, (min_joint, max_joint, init_theta) in enumerate(zip(min_joints, max_joints, init_thetas)):
                # 関節情報を取得して，スライダーの追加
                slider = p.addUserDebugParameter(f"joiint {joint_idx + 1}", min_joint, max_joint, init_theta)
                sliders.append(slider)

        else:
            # 異常
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        # プロパティの更新
        self._interpolation = interpolation
        self._sliders = sliders

    def _get_joint_limit(self):
        """
        関節の限界値 (最小値 + 最大値) を取得

        戻り値
            min_joints (list): 関節の最小値 [rad]
            max_joints (list): 関節の最大値 [rad]
        """
        min_joints = []
        max_joints = []

        # 全関節の最小値と最大値を取得
        for i in range(self._n_joints):
            # 関節に関する情報を取得
            joint_info = p.getJointInfo(self._robot_id, i)
            min_joint  = joint_info[self._IDX_MIN_JOINT]
            max_joint  = joint_info[self._IDX_MAX_JOINT]
            # 最小値と最大値をリストに保存
            min_joints.append(min_joint)
            max_joints.append(max_joint)

        return min_joints, max_joints

    def _get_init_thetas(self):
        """
        初期角度を取得

        戻り値
            init_thetas(numpy.ndarray): 初期角度 [rad]
        """
        init_thetas = np.ones(self._n_joints) * self._JOINT_INIT

        return init_thetas


    def run(self):
        """
        実行
            スライダー内の関節角度を取得して，シミュレータ上のロボットを動かす
        """
        # 文字列の出力ID
        text_id = p.addUserDebugText("", textPosition=[0, 0, 0])

        # リアルタイムでのシミュレーション
        p.setRealTimeSimulation(1)

        while True:
            # スライダー内の値を取得
            slider_values = self._get_slider_values()

            # 関節角度を設定
            self._set_joint(slider_values)

            # 値をGUIに設定
            self._set_text(text_id)

            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _get_slider_values(self):
        """
        スライダー内の値を取得

        戻り値
            slider_values(numpy.ndarray): スライダー内の情報
        """
        slider_values = []

        for slider in self._sliders:
            # スライダー1つずつ値を取得
            slider_value = p.readUserDebugParameter(slider)
            slider_values.append(slider_value)

        return np.array(slider_values)

    def _set_joint(self, thetas):
        """
        関節角度の設定

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

    def _set_text(self, text_id):
        """
        GUIにテキストを設定
        
        パラメータ
            text_id(): 設定したテキストID
        """
        # エンドエフェクタの位置を取得
        ee_pos = p.getLinkState(self._robot_id, self._n_joints)[0]
        text = f"end effecter pos:\nx={ee_pos[0]:.2f}, y={ee_pos[1]:.2f}, z={ee_pos[2]:.2f}"

        p.addUserDebugText(text, ee_pos, textColorRGB=[0, 0, 0], textSize=self._DEBUG_TEXT_SIZE, lifeTime=self._DEBUG_TEXT_LIFE_TIME, replaceItemUniqueId=text_id)
