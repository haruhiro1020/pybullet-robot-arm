import numpy as np

from pybullet_robot import MainPyBulletRobot
from constant import *


N_ROBOT_AXIS = DIMENTION_3D
CONST_SEED   = 1
HAND_FLG     = True


def main():
    if N_ROBOT_AXIS == DIMENTION_2D:
        environment_urdf = "environment_2dof.urdf"
    elif N_ROBOT_AXIS == DIMENTION_3D:
        environment_urdf = "environment_3dof.urdf"
    else:
        raise ValueError(f"N_ROBOT_AXIS is abnormal. N_ROBOT_AXIS is {N_ROBOT_AXIS}")

    path_plan     = PATHPLAN.RRT.value
    grasp_urdf    = "grasp_object.urdf"
    interpolation = INTERPOLATION.POSITION.value

    my_robot = MainPyBulletRobot(interpolation, N_ROBOT_AXIS, environment_urdf=environment_urdf,
                                 grasp_urdf=grasp_urdf, hand=HAND_FLG)

    np.random.seed(CONST_SEED)

    if N_ROBOT_AXIS == DIMENTION_2D:
        start_pos = np.array([1.0, -1.0])
    else:
        start_pos = np.array([1.0, -1.0, 1.0])

    if interpolation == INTERPOLATION.JOINT.value:
        start_theta = my_robot.convert_pos_to_theta(start_pos)
        print(f"start_theta = {start_theta}")
        result = my_robot.run(start_theta, path_plan)
    else:
        result = my_robot.run(start_pos, path_plan)

    print(f"result = {result}")


if __name__ == "__main__":
    main()
