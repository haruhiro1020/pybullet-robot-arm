import numpy as np

from pybullet_main import MainPyBulletRobot
from constant import *


N_ROBOT_AXIS = DIMENTION_6D
CONST_SEED   = 1
HAND_FLG     = True


def main():
    environment_urdf = "environment_6dof.urdf"
    path_plan        = PATHPLAN.RRT.value
    grasp_urdf       = "grasp_object.urdf"
    interpolation    = INTERPOLATION.POSITION.value
    n_cameras        = CAMERANUM.MULTI.value

    my_robot = MainPyBulletRobot(interpolation, N_ROBOT_AXIS, environment_urdf, grasp_urdf, n_cameras, hand=HAND_FLG)

    np.random.seed(CONST_SEED)

    start_pos = np.array([1.5, -0.5, 1.5, np.pi/4, np.pi/2, 0])

    if interpolation == INTERPOLATION.JOINT.value:
        start_theta = my_robot.convert_pos_to_theta(start_pos)
        print(f"start_theta = {start_theta}")
        result = my_robot.run(start_theta, path_plan)
    else:
        result = my_robot.run(start_pos, path_plan)

    print(f"result = {result}")


if __name__ == "__main__":
    main()
