import pybullet as p
import pybullet_data
import time
import numpy as np

from constant import *


class MainPyBulletRobot:
    _PLANE_URDF     = "plane.urdf"
    _IDX_MIN_JOINT  = 8
    _IDX_MAX_JOINT  = 9
    _JOINT_INIT     = 0.0
    _SLIDER_MAKE_WAIT_TIME = 0.2
    _SIMULATION_SLEEP_TIME = 1. / 240.
    _DEBUG_TEXT_LIFE_TIME  = 0
    _DEBUG_TEXT_SIZE       = 0.5
    _ZERO_NEAR = 1e-4


    def __init__(self, robot_urdf, interpolation):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -GRABITY_VALUE)

        self._init_robot(robot_urdf)
        self._init_environment()
        self._init_sliders(interpolation)

        time.sleep(self._SLIDER_MAKE_WAIT_TIME)


    def _init_robot(self, robot_urdf):
        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)
        self._n_joints = p.getNumJoints(self._robot_id) - 1

        if not (self._n_joints == DIMENTION_2D or self._n_joints == DIMENTION_3D):
            raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")

    def _init_environment(self):
        p.loadURDF(self._PLANE_URDF)

    def _init_sliders(self, interpolation):
        sliders = []

        if interpolation == INTERPOLATION.JOINT.value:
            min_joints, max_joints = self._get_joint_limit()
            init_thetas = self._get_init_thetas()
            for joint_idx, (mn, mx, init) in enumerate(zip(min_joints, max_joints, init_thetas)):
                slider = p.addUserDebugParameter(f"joint {joint_idx + 1}", mn, mx, init)
                sliders.append(slider)

        elif interpolation == INTERPOLATION.POSITION.value:
            slider_names = ["X", "Y", "Z"]
            min_positions, max_positions = self._get_pose_limit()
            init_pose = self._get_init_pose()
            for idx, (mn, mx, init_p) in enumerate(zip(min_positions, max_positions, init_pose)):
                slider = p.addUserDebugParameter(slider_names[idx], mn, mx, init_p)
                sliders.append(slider)

        else:
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        self._interpolation = interpolation
        self._sliders = sliders

    def _get_joint_limit(self):
        min_joints, max_joints = [], []
        for i in range(self._n_joints):
            joint_info = p.getJointInfo(self._robot_id, i)
            min_joints.append(joint_info[self._IDX_MIN_JOINT])
            max_joints.append(joint_info[self._IDX_MAX_JOINT])
        return min_joints, max_joints

    def _get_init_thetas(self):
        return np.ones(self._n_joints) * self._JOINT_INIT

    def _get_pose_limit(self):
        abs_max = 3.5 if self._n_joints == DIMENTION_3D else 2.5
        min_pose = np.ones(self._n_joints + 1) * abs_max * -1
        max_pose = np.ones(self._n_joints + 1) * abs_max
        if self._n_joints == DIMENTION_2D:
            min_pose[self._n_joints] = -self._ZERO_NEAR
            max_pose[self._n_joints] =  self._ZERO_NEAR
        return min_pose, max_pose

    def _get_init_pose(self):
        if self._n_joints == DIMENTION_3D:
            return np.array([0.5, 0.5, 1.0])
        return np.array([0.5, 0.5, 0.0])


    def run(self):
        text_id = p.addUserDebugText("", textPosition=[0, 0, 0])
        p.setRealTimeSimulation(1)

        while True:
            slider_values = self._get_slider_values()
            thetas = self._convert_pos_to_theta(slider_values)
            self._set_joint(thetas)
            self._set_text(text_id, thetas)
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _convert_pos_to_theta(self, pos):
        if self._interpolation == INTERPOLATION.POSITION.value:
            thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
            return np.array(thetas)
        return np.copy(pos)

    def _get_slider_values(self):
        return np.array([p.readUserDebugParameter(s) for s in self._sliders])

    def _set_joint(self, thetas):
        for i in range(len(thetas)):
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=thetas[i]
            )

    def _set_text(self, text_id, thetas):
        ee_pos = p.getLinkState(self._robot_id, self._n_joints)[0]
        if self._interpolation == INTERPOLATION.POSITION.value:
            text = "thetas:\n" + "".join(f"joint{i+1}={thetas[i]:.2f}\n" for i in range(thetas.size))
        else:
            text = f"end effecter pos:\nx={ee_pos[0]:.2f}, y={ee_pos[1]:.2f}, z={ee_pos[2]:.2f}"
        p.addUserDebugText(text, ee_pos, textColorRGB=[0, 0, 0], textSize=self._DEBUG_TEXT_SIZE,
                           lifeTime=self._DEBUG_TEXT_LIFE_TIME, replaceItemUniqueId=text_id)
