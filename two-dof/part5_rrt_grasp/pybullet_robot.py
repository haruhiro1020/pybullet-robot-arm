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


class MainPyBulletRobot:
    # 定数の定義
    _PLANE_URDF     = "plane.urdf"  # 地面に関する urdf ファイル
    
    _IDX_MIN_JOINT  = 8             # 関節の最小値が保存されている要素番号
    _IDX_MAX_JOINT  = 9             # 関節の最大値が保存されている要素番号
    
    _SIMULATION_SLEEP_TIME = 1. / 200.   # シミュレーションの待機時間 [sec]
    
    _INTERFERENCE_MARGIN   = 0.1    # 干渉判定のマージン [m]
    
    _PATH_PLAN_TIME = 1000      # 経路生成の最大時間 [sec]
    _N_MARGIN_MOVE  = 50        # 経路生成終了後の余白時間 [回]
    
    _N_HAND_JOINT   = 4         # ハンド用の関節数
    _N_INVERSE_HAND_JOINT = -2  # 逆運動学計算時のハンド関節数
    
    _GRASP_OBJECT_POS    = [ 1.8, 1.0, 0.05]    # 把持対象物の位置
    _GRASP_OBJECT_OFFSET = [-0.4, 0,   0]       # 把持対象物の位置のオフセット
    
    
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

        self._rrt = None


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
        self._grasp_constraint_id = None
        if grasp_urdf is not None:
            self._grasp_id = p.loadURDF(grasp_urdf, basePosition=[pos + offset for pos, offset in zip(self._GRASP_OBJECT_POS, self._GRASP_OBJECT_OFFSET)])

            # 把持対象物に摩擦を付与する
            p.changeDynamics(self._grasp_id,        # 把持対象物ID
                            -1,                     # ベースに対して
                            lateralFriction=1.0,    # 床との摩擦係数
                            spinningFriction=1.0,   # 回転摩擦係数
                            rollingFriction=1)      # 転がり摩擦

            # 把持対象物の位置・姿勢を取得
            grasp_pos, grasp_ori = self._get_grasp_pos()
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
            bool: True/False = 経路生成に成功/失敗
        """
        # リアルタイムでのシミュレーション
        p.setRealTimeSimulation(1)

        # 把持対象物の位置を取得
        end_pos, _ = self._get_grasp_pos(dim2=True)
        end_pos = np.array(end_pos)

        # 始点と終点で干渉していないかの確認
        self._is_interference_start_end_pos(start_pos, end_pos)

        # 経路生成手法の設定
        self._set_path_plan(path_plan)

        # 経路生成の実装
        result = self._path_planning(start_pos, end_pos)

        if result:      # 経路生成に成功
            self._post_path_planning(start_pos, end_pos)

        # ファイル保存
        self._rrt.save()

        # 経路生成後の余白時間
        self._exec_margin_time(end_pos)

        return result

    def _exec_margin_time(self, end_pos):
        """
        経路生成後の余白時間の処理

        パラメータ
            end_pos(numpy.ndarray): 経路生成の終点
        """
        # 長めの待機時間
        time.sleep(self._SIMULATION_SLEEP_TIME * 10)

        # PyBulletで関節角度を与えても，与えた関節角度ピッタリにはならず，数値誤差が発生するから，終点へ移動する処理を実施する
        end_theta = self._convert_pos_to_theta(end_pos)

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self._N_MARGIN_MOVE):
            # 終点に移動
            self._set_joint(end_theta)
            # グリッパーの実行
            self._run_gripper(open=True)
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
            self._set_joint(end_theta)
            # グリッパーの実行
            self._run_gripper(close=True)
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

    def _get_robot_joint_from_grasp_pos(self):
        """
        把持対象物の位置となるロボットの関節角度を取得
        
        戻り値
            numpy.ndarray: ロボットの関節角度
        """
        # 把持対象物の位置を取得
        grasp_pos, grasp_ori = self._get_grasp_pos()
        # 逆運動学 (手先位置から関節角度へ変換) の実施
        thetas = self._convert_pos_to_theta(grasp_pos)

        return thetas

    def _get_grasp_pos(self, dim2=False):
        """
        把持対象物の位置を取得

        パラメータ
            dim2(bool): 2次元位置として取得するかどうか

        戻り値
            list: 把持対象物の位置 ([rad] or [m])
            list: 把持対象物の姿勢
        """
        if self._grasp_id is None:  # 把持対象物が存在しない
            raise ValueError("self._grasp_id is abnorma. please set grasp_urdf.")

        # 把持対象物の位置[m]を取得
        grasp_pos, grasp_ori = p.getBasePositionAndOrientation(self._grasp_id)

        if dim2:
            grasp_pos = grasp_pos[:DIMENTION_2D]

        return grasp_pos, grasp_ori

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
        # グリッパーの実行
        self._run_gripper(open=True)

        # 始点から終点までの経路を移動
        for row_idx in range(self._rrt.pathes.shape[0]):
            next_theta = self._convert_pos_to_theta(self._rrt.pathes[row_idx])
            self._set_joint(next_theta)
            # グリッパーの実行
            self._run_gripper(open=True)
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

    def _run_gripper(self, open=False, close=False):
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

        # グリッパーの実行
        self._run_gripper(open=True)

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

        # グリッパーの実行
        self._run_gripper(open=True)

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
        thetas = np.array(thetas)[:-self._N_INVERSE_HAND_JOINT]

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
