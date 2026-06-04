from pybullet_robot import MainPyBulletRobot
from constant import *


def main():
    robot_urdf    = "robot_3dof.urdf"
    interpolation = INTERPOLATION.JOINT.value

    my_robot = MainPyBulletRobot(robot_urdf, interpolation)
    my_robot.run()


if __name__ == "__main__":
    main()
