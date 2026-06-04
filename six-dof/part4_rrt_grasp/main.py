# Pybullet (Pythonでの3次元物理シミュレータ) による2軸ロボットアームの可視化


# 標準ライブラリの読み込み
import numpy as np


# 自作モジュールの読み込み
from pybullet_robot import MainPyBulletRobot
from constant import *



N_ROBOT_AXIS = DIMENTION_6D     # ロボットの関節数
CONST_SEED = 1  # シード値 (常に同じ結果としたいから)
HAND_FLG   = True   # ハンドの装着有無


def main():
    """
    メイン処理
    """
    # 環境が保存されている URDF ファイル名・探索空間・初期位置
    if N_ROBOT_AXIS == DIMENTION_2D:    # 2軸ロボットアーム
        environment_urdf = "environment_2dof.urdf"
        interpolation = INTERPOLATION.POSITION.value    # 直交空間の探索
        start_pos = np.array([1.0, -1.0])

    elif N_ROBOT_AXIS == DIMENTION_3D:  # 3軸ロボットアーム
        environment_urdf = "environment_3dof.urdf"
        interpolation = INTERPOLATION.POSITION.value    # 直交空間の探索
        start_pos = np.array([1.0, -1.0, 1.0])

    elif N_ROBOT_AXIS == DIMENTION_6D:  # 6軸ロボットアーム
        environment_urdf = "environment_6dof.urdf"
        interpolation = INTERPOLATION.JOINT.value       # 関節空間の探索
        start_pos = np.array([1.0, -1.0, 1.0, 0, np.pi/2, 0]) # 位置(x, y, z)・姿勢(ロール, ピッチ, ヨー)

    else:
        raise ValueError(f"N_ROBOT_AXIS is abnormal. N_ROBOT_AXIS is {N_ROBOT_AXIS}")

    # 経路生成手法
    path_plan = PATHPLAN.RRT.value

    # 把持対象物が保存されている URDF ファイル名
    grasp_urdf = "grasp_object.urdf"

    # Pybulletを使用するインスタンス作成
    my_robot = MainPyBulletRobot(interpolation, N_ROBOT_AXIS, environment_urdf=environment_urdf, grasp_urdf=grasp_urdf, hand=HAND_FLG)

    # シード値の設定
    np.random.seed(CONST_SEED)

    if interpolation == INTERPOLATION.JOINT.value:
        # 関節空間の探索時は位置を関節角度に変換
        start_theta = my_robot.convert_pos_to_theta(start_pos)
        print(f"start_theta = {start_theta}")

        result = my_robot.run(start_theta, path_plan)
    else:
        result = my_robot.run(start_pos,   path_plan)

    print(f"result = {result}")


if __name__ == "__main__":
    # 本ファイルがメインで呼ばれた時の処理
    main()

