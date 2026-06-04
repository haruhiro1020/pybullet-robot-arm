# Pybullet (Pythonでの3次元物理シミュレータ) による2軸ロボットアームの可視化


# 標準ライブラリの読み込み
import numpy as np


# 自作モジュールの読み込み
from pybullet_main import MainPyBulletRobot
from constant import *



CONST_SEED = 1      # シード値 (常に同じ結果としたいから)
HAND_FLG   = True   # ハンドの装着有無


def main():
    """
    メイン処理
    """
    # 経路生成手法
    path_plan = PATHPLAN.RRT.value          # RRTによる経路生成

    # 環境が保存されている URDF ファイル名
    environment_urdf = "environment.urdf"

    # 把持対象物が保存されている URDF ファイル名
    grasp_urdf = "grasp_object.urdf"

    # 探索空間を指定
    # interpolation = INTERPOLATION.JOINT.value       # 関節空間の探索
    interpolation = INTERPOLATION.POSITION.value    # 直交空間の探索

    # カメラ数を指定
    # n_cameras = CAMERANUM.SINGLE.value              # 1台のカメラ
    n_cameras = CAMERANUM.MULTI.value               # 複数台のカメラ

    # Pybulletを使用するインスタンス作成
    my_robot  = MainPyBulletRobot(interpolation, DIMENTION_2D, environment_urdf, grasp_urdf, n_cameras, hand=HAND_FLG)

    # 初期位置・終点位置
    start_pos = np.array([1.0, -1.0])

    # シード値の設定
    np.random.seed(CONST_SEED)

    if interpolation == INTERPOLATION.JOINT.value:
        # 関節空間の探索時は位置を関節角度に変換
        start_theta = my_robot.convert_pos_to_theta(start_pos)
        print(f"start_theta = {start_theta}")

        result = my_robot.run(start_theta, path_plan)
    else:
        result = my_robot.run(start_pos, path_plan)

    print(f"result = {result}")


if __name__ == "__main__":
    # 本ファイルがメインで呼ばれた時の処理
    main()
