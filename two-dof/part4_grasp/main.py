# Pybullet (Pythonでの3次元物理シミュレータ) による2軸ロボットアームの可視化


# 自作モジュールの読み込み
from pybullet_robot import MainPyBulletRobot
from constant import *



HAND_FLG = True   # ハンドの装着有無


def main():
    """
    メイン処理
    """
    # ロボットが保存されている URDF ファイル名
    if HAND_FLG:
        # ハンド付きのURDF
        robot_urdf = "robot_2dof_hand.urdf"
    else:
        # ハンドなしのURDF
        robot_urdf = "robot_2dof.urdf"

    # 把持対象物が保存されている URDF ファイル名
    grasp_urdf = "grasp_object.urdf"

    # 探索空間を指定
    interpolation = INTERPOLATION.POSITION.value    # 直交空間の探索

    # PyBulletを使用するインスタンス作成
    my_robot = MainPyBulletRobot(interpolation, robot_urdf, grasp_urdf=grasp_urdf, hand=HAND_FLG)

    # PyBullet上のロボットを動かす
    result = my_robot.run()

    print(f"result = {result}")


if __name__ == "__main__":
    # 本ファイルがメインで呼ばれた時の処理
    main()
