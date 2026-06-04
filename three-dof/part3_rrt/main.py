import numpy as np

from pybullet_robot import MainPyBulletRobot
from constant import *


N_ROBOT_AXIS = DIMENTION_3D
CONST_SEED   = 1


def main():
    if N_ROBOT_AXIS == DIMENTION_2D:
        robot_urdf = "robot_2dof.urdf"
    elif N_ROBOT_AXIS == DIMENTION_3D:
        robot_urdf = "robot_3dof.urdf"
    else:
        raise ValueError(f"N_ROBOT_AXIS is abnormal. N_ROBOT_AXIS is {N_ROBOT_AXIS}")

    environment_urdf = "environment.urdf"
    path_plan        = PATHPLAN.RRT.value
    interpolation    = INTERPOLATION.JOINT.value

    my_robot = MainPyBulletRobot(robot_urdf, environment_urdf, interpolation)
    np.random.seed(CONST_SEED)

    if N_ROBOT_AXIS == DIMENTION_2D:
        start_pos = np.array([1.0, -1.0])
        end_pos   = np.array([1.0,  1.0])
    else:
        start_pos = np.array([1.0, -1.0, 1.0])
        end_pos   = np.array([1.0,  1.0, 1.0])

    if interpolation == INTERPOLATION.JOINT.value:
        start_theta = my_robot.convert_pos_to_theta(start_pos)
        end_theta   = my_robot.convert_pos_to_theta(end_pos)
        print(f"start_theta = {start_theta}")
        print(f"end_theta = {end_theta}")
        result = my_robot.run(start_theta, end_theta, path_plan)
    else:
        result = my_robot.run(start_pos, end_pos, path_plan)

    print(f"result = {result}")


if __name__ == "__main__":
    main()
