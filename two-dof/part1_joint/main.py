# 自作モジュールの読み込み
from pybullet_robot import MainPyBulletRobot
from constant import *



def main():
    """
    メイン処理
    """
    # ロボットが保存されている URDF ファイル名
    robot_urdf = "robot_2dof.urdf"

    # 探索空間を指定
    interpolation = INTERPOLATION.JOINT.value

    # Pybulletを使用するインスタンス作成
    my_robot = MainPyBulletRobot(robot_urdf, interpolation)
    my_robot.run()


if __name__ == "__main__":
    # 本ファイルがメインで呼ばれた時の処理
    main()

