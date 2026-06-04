# PyBulletで使用するロボットを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import pybullet_data    # PyBulletで使用するデータ
import time             # 時間
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *
from pybullet_rrt import RRTPyBullet



class MainPyBulletRobot:
    # 定数の定義
    _PLANE_URDF     = "plane.urdf"  # 地面に関する urdf ファイル
    
    _IDX_MIN_JOINT  = 8             # 関節の最小値が保存されている要素番号
    _IDX_MAX_JOINT  = 9             # 関節の最大値が保存されている要素番号
    
    _SIMULATION_SLEEP_TIME = 0.05   # シミュレーションの待機時間 [sec]
    
    _INTERFERENCE_MARGIN   = 0.1    # 干渉判定のマージン [m]
    
    _PATH_PLAN_TIME = 1000   # 経路生成の最大時間 [sec]
    _N_MARGIN_MOVE  = 100   # 経路生成終了後の余白時間 [回]
    
    
    def __init__(self, robot_urdf, environment_urdf, interpolation):
        """
        コンストラクタ

        パラメータ
            robot_urdf(str): ロボットアームのファイル名 (urdf)
            environment_urdf(str): 環境のファイル名 (urdf)
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
        self._init_robot(robot_urdf, interpolation)

        # 環境の初期化
        self._init_environment(environment_urdf)

        self._rrt = None


    def _init_robot(self, robot_urdf, interpolation):
        """
        ロボットの初期化

        パラメータ
            robot_urdf(str): ロボットアームのファイル名 (urdf)
            interpolation(str): 探索方法 (関節空間/位置空間)
        """
        # 引数の確認
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            # 異常
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        # プロパティの更新
        self._interpolation = interpolation

        # ロボットを読み込む．ベースリンクの原点は (x, y, z) = (0, 0, 0) として，ベースリンクは地面に固定
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # urdf よりロボットの関節数を取得 (エンドエフェクタ用の仮想関節は不要なため -1)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        # ロボットの関節数に応じて，robot.py内のクラスを決定
        if self._n_joints != DIMENTION_2D:
            # 2軸ロボットアーム以外
            raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

    def _init_environment(self, environment_urdf):
        """
        環境の初期化
        
        パラメータ
            environment_urdf(str): 環境が保存されているファイル名
        """
        # 地面を読み込む (pybulletが提供している "plane.urdf" を読み込む)
        p.loadURDF(self._PLANE_URDF)

        # 環境を読み込む
        self._environment_id = p.loadURDF(environment_urdf, basePosition=[0, 0, 0], useFixedBase=True)


    def run(self, start_pos, end_pos, path_plan):
        """
        実行
            始点から終点まで，干渉しない経路を生成

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            path_plan(str): 経路生成手法
        
        戻り値
            result(bool): True/False = 経路生成に成功/失敗
        """
        # リアルタイムでのシミュレーション
        p.setRealTimeSimulation(1)

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

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self._N_MARGIN_MOVE):
            # 終点に移動
            # PyBulletで関節角度を与えても，与えた関節角度ピッタリにはならず，数値誤差が発生するから，終点へ移動する処理を実施する
            end_theta = self._convert_pos_to_theta(end_pos)
            self._set_joint(end_theta)
            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)

        return result

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

        start_time = time.time()

        # 始点から終点までの経路が生成するまでループ
        while True:
            now_time = time.time()
            if (now_time - start_time) >= self._PATH_PLAN_TIME:
                # タイムアウト
                break

            # 経路生成を1度実行
            new_node_pos, near_node_pos, near_node = self._rrt.expand_once(end_pos)
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
        theta = self._convert_pos_to_theta(start_pos)
        self._set_jump_joint(theta)

        # print(f"self._rrt.pathes = {self._rrt.pathes}")

        # 始点から終点までの経路を移動
        for row_idx in range(self._rrt.pathes.shape[0]):
            next_theta = self._convert_pos_to_theta(self._rrt.pathes[row_idx])
            self._set_joint(next_theta)
            # 待機時間
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _set_jump_joint(self, thetas):
        """
        関節角度をジャンプ

        パラメータ
            thetas(numpy.ndarray): 関節角度
        """
        for i in range(len(thetas)):
            # 関節角度を設定
            p.resetJointState(
                bodyUniqueId=self._robot_id,    # IDの設定
                jointIndex=i,                   # 関節番号の設定
                targetValue=thetas[i]           # 関節角度
            )

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
        theta = self._convert_pos_to_theta(pos2)
        self._set_joint(theta)

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
        theta = self._convert_pos_to_theta(pos)
        # print(f"theta = {theta}")
        # 位置にジャンプ
        self._set_jump_joint(theta)

        # 待機時間
        time.sleep(self._SIMULATION_SLEEP_TIME)

        # ロボットと干渉物との干渉判定
        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:  # 干渉なし
            is_interference = False

        return is_interference


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

