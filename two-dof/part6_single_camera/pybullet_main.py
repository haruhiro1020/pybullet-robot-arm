# PyBulletのメイン処理を記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import pybullet_data    # PyBulletで使用するデータ
import time             # 時間
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *
from pybullet_rrt import PyBulletRRTController          # 経路生成に関して
from pybullet_environment import PyBulletEnvironment    # 環境に関して
from pybullet_grasp import PyBulletGraspObject          # 把持物体に関して
from pybullet_robot import PyBulletRobotController      # ロボットに関して
from pybullet_interference import PyBulletInteference   # 干渉判定に関して
from pybullet_camera import PyBulletCamera              # カメラに関して



class MainPyBulletRobot:
    # 定数の定義
    __SIMULATION_SLEEP_TIME = 0.05  # シミュレーションの待機時間 [sec]
    __N_MARGIN_MOVE  = 20           # 経路生成終了後の余白時間 [回]


    def __init__(self, interpolation, n_robot_joint, environment_urdf, grasp_urdf, camera_urdf, hand=False):
        """
        コンストラクタ

        パラメータ
            interpolation(str): 補間方法 (関節空間/位置空間)
            n_robot_joint(int): ロボットアームの関節数(2, 3, 6だけ)
            environment_urdf(str): 環境のファイル名 (urdf)
            grasp_urdf(str): 把持物体のファイル名 (urdf)
            camera_urdf(str): カメラのファイル名 (urdf)
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
        self.__robot = PyBulletRobotController(n_robot_joint, interpolation, hand)

        # 環境の初期化
        self.__environment = PyBulletEnvironment(environment_urdf, self.__robot.n_robot_joint)

        # 把持物体の初期化
        self.__grasp_obj = PyBulletGraspObject(grasp_urdf, self.__robot.n_robot_joint)

        # 干渉判定の初期化
        self.__interference = PyBulletInteference(self.__robot, self.__environment)

        # カメラの初期化
        self.__camera = PyBulletCamera(camera_urdf)

        # 経路生成の初期化
        self.__rrt = None


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

        # 把持物体の位置を取得
        end_pos = self.__get_grasp_pos_from_camera()
        print(f"end_pos = {end_pos}")

        # 経路生成手法の設定
        self.__rrt = PyBulletRRTController(path_plan)

        # 経路生成の実装
        result = self.__rrt.planning(start_pos, end_pos, self.__robot.interpolation, self.__interference, self.__robot.joints_limit, self.__robot.weight_joint)

        print(f"self.__rrt.planning() = {result}")

        if result:
            # 経路生成に成功
            self.__post_path_planning(start_pos, end_pos)

        # ファイル保存
        self.__rrt.save()

        # 経路生成後の余白時間
        self.__exec_margin_time()

        return result


    def __get_grasp_pos_from_direct(self, offset=False):
        """
        把持物体の位置を取得 (直接)

        パラメータ
            offset(bool): オフセットの有無

        戻り値
            numpy.ndarray: 把持物体の位置
        """
        # 把持物体の位置を取得
        pos, _ = self.__grasp_obj.get_grasp_pos(offset=offset, dim2=True)

        # Numpy型に型変換
        pos = np.array(pos)

        return pos


    def __get_grasp_pos_from_camera(self):
        """
        カメラから把持物体の位置を取得
        
        戻り値
            numpy.ndarray: 把持物体の位置
        """
        # カメラから物体位置(3次元)の取得
        pos = self.__camera.get_pos(self.__grasp_obj.grasp_obj_id)
        # 今回は2軸ロボットアームだけに対応するため，物体位置を2次元に変換する
        pos = pos[:DIMENTION_2D]
        # 探索方法に応じた終点に変換
        if self.__interference == INTERPOLATION.JOINT:
            # 逆運動学により，関節角度を取得
            pos = self.__robot.convert_pos_to_theta(pos)

        return pos


    def __exec_margin_time(self):
        """
        経路生成後の余白時間の処理
        """
        # 把持物体の位置を取得 (直接)
        # end_pos = self.__get_grasp_pos_from_direct(offset=False)

        # 把持物体の位置をカメラから取得
        end_pos = self.__get_grasp_pos_from_camera()

        # 把持物体への位置へ移動
        end_theta = self.__robot.convert_pos_to_theta(end_pos)

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self.__N_MARGIN_MOVE):
            # 終点に移動
            self.__robot.set_joint(end_theta)
            # グリッパーの実行
            self.__robot.run_gripper(open=True)
            # 待機時間
            time.sleep(self.__SIMULATION_SLEEP_TIME)

        # 把持物体の拘束を解除
        self.__grasp_obj.release_constraint()
        time.sleep(self.__SIMULATION_SLEEP_TIME)

        # 経路生成後の余白時間 (即座にPyBulletが終了するのを防ぐための余白時間)
        for _ in range(self.__N_MARGIN_MOVE):
            # 終点に移動
            self.__robot.set_joint(end_theta)
            # グリッパーの実行
            self.__robot.run_gripper(close=True)
            # 待機時間
            time.sleep(self.__SIMULATION_SLEEP_TIME)


    def __post_path_planning(self, start_pos, end_pos):
        """
        経路生成の後処理 (経路生成成功時だけ実装)

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
        """
        # 始点に移動
        theta = self.__robot.convert_pos_to_theta(start_pos)
        self.__robot.set_jump_joint(theta)
        # グリッパーの実行
        self.__robot.run_gripper(open=True)

        # 始点から終点までの経路のログ出力
        for row_idx in range(self.__rrt.pathes.shape[0] - 1):
            now_data  = self.__rrt.pathes[row_idx]
            next_data = self.__rrt.pathes[row_idx + 1]
            diff = next_data - now_data
            distance = np.linalg.norm(diff)

        # 始点から終点までの経路を移動
        for row_idx in range(self.__rrt.pathes.shape[0]):
            next_theta = self.__robot.convert_pos_to_theta(self.__rrt.pathes[row_idx])
            self.__robot.set_joint(next_theta)
            # グリッパーの実行
            self.__robot.run_gripper(open=True)
            # 待機時間
            time.sleep(self.__SIMULATION_SLEEP_TIME)


    def convert_pos_to_theta(self, pos):
        """
        位置から関節角度に変換

        パラメータ
            pos(numpy.ndarray): 位置 / 関節角度

        戻り値
            thetas(numpy.ndarray): 関節角度
        """
        thetas = self.__robot.convert_pos_to_theta(pos, force=True)

        return thetas
