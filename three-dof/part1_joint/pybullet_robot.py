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

            for joint_idx, (min_joint, max_joint, init_theta) in enumerate(zip(min_joints, max_joints, init_thetas)):
                slider = p.addUserDebugParameter(f"joint {joint_idx + 1}", min_joint, max_joint, init_theta)
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


    def run(self):
        text_id = p.addUserDebugText("", textPosition=[0, 0, 0])
        p.setRealTimeSimulation(1)

        while True:
            slider_values = self._get_slider_values()
            self._set_joint(slider_values)
            self._set_text(text_id)
            time.sleep(self._SIMULATION_SLEEP_TIME)

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

    def _set_text(self, text_id):
        ee_pos = p.getLinkState(self._robot_id, self._n_joints)[0]
        text = f"end effecter pos:\nx={ee_pos[0]:.2f}, y={ee_pos[1]:.2f}, z={ee_pos[2]:.2f}"
        p.addUserDebugText(text, ee_pos, textColorRGB=[0, 0, 0], textSize=self._DEBUG_TEXT_SIZE,
                           lifeTime=self._DEBUG_TEXT_LIFE_TIME, replaceItemUniqueId=text_id)
