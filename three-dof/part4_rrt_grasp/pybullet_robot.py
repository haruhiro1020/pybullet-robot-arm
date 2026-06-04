import pybullet as p
import pybullet_data
import time
import numpy as np

from constant import *
from pybullet_rrt import RRTPyBullet


class BaseGripper:
    _JOINT_CURRENT_VALUE_IDX = 0

    def __init__(self, robot_id, n_joint):
        self._robot_id = robot_id
        self._n_joint  = n_joint

    def run(self):
        raise InterruptedError("run() is necessary override.")


class ParallelGripper(BaseGripper):
    _GRIPPER_RIGHT_IDX  =  -2
    _GRIPPER_LEFT_IDX   =  -1
    _GRIPPER_CLOSE_VAL  = 0.03
    _GRIPPER_OPEN_VAL   = 0

    def __init__(self, robot_id, n_joint):
        super().__init__(robot_id, n_joint)
        self._chg_dynamics()

    def _chg_dynamics(self):
        for idx in self._get_gripper_right_left_idx():
            p.changeDynamics(self._robot_id, idx, lateralFriction=1.0)

    def run(self, open=False, close=False):
        joint_values = self._get_joint_values()

        if open:
            direction    = self._get_move_direction(open=True)
            joint_values = direction * self._GRIPPER_OPEN_VAL
        elif close:
            direction    = self._get_move_direction(open=False)
            joint_values = direction * self._GRIPPER_CLOSE_VAL

        self._set_joint_values(joint_values)

    def _get_move_direction(self, open):
        move_direction = np.array([-1.0, 1.0])
        if not open:
            move_direction *= -1
        return move_direction

    def _get_gripper_right_left_idx(self):
        return [self._n_joint + self._GRIPPER_RIGHT_IDX, self._n_joint + self._GRIPPER_LEFT_IDX]

    def _set_joint_values(self, values):
        if values.shape[0] != DIMENTION_2D:
            raise ValueError(f"values'shape[0] is abnormal. values'shape[0] is {values.shape[0]}")
        for idx, gripper_idx in enumerate(self._get_gripper_right_left_idx()):
            p.setJointMotorControl2(
                bodyIndex=self._robot_id, jointIndex=gripper_idx,
                controlMode=p.POSITION_CONTROL, targetPosition=values[idx],
                positionGain=0.5, velocityGain=1.0
            )

    def _get_joint_values(self):
        joint_values = []
        for gripper_idx in self._get_gripper_right_left_idx():
            joint_state = p.getJointState(bodyUniqueId=self._robot_id, jointIndex=gripper_idx)
            joint_values.append(joint_state[self._JOINT_CURRENT_VALUE_IDX])
        return np.array(joint_values)


class PyBulletRobot:
    _DIM_JOINT        = DIMENTION_NONE
    _DIM_POSE         = DIMENTION_NONE
    _N_HAND_JOINT     = 4
    _N_HAND_JOINT_NOT_FIXED = 2

    def __init__(self, robot_id, interpolation, hand):
        if not (interpolation == INTERPOLATION.JOINT.value or interpolation == INTERPOLATION.POSITION.value):
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        self._robot_id      = robot_id
        self._interpolation = interpolation
        self._n_joints      = p.getNumJoints(robot_id) - 1

        if hand:
            if self._n_joints != (self._DIM_JOINT + self._N_HAND_JOINT):
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")
            self._hand = ParallelGripper(self._robot_id, self._n_joints)
        else:
            if self._n_joints != self._DIM_JOINT:
                raise ValueError(f"self._n_joints is abnormal. {self._n_joints} is abnormal.")
            self._hand = None

    def _chk_pos_dim(self, pos):
        if len(pos) != self._DIM_POSE:
            raise ValueError(f"pos's shape is abnormal. pos'size is {len(pos)}")

    def _chk_thetas_dim(self, thetas):
        if self._hand is None:
            if len(thetas) != self._DIM_JOINT:
                raise ValueError(f"theta's shape is abnormal. thetas'size is {len(thetas)}")
        else:
            if len(thetas) != (self._DIM_JOINT + self._N_HAND_JOINT_NOT_FIXED):
                raise ValueError(f"theta's shape is abnormal. thetas'size is {len(thetas)}")

    def set_joint(self, thetas):
        self._chk_thetas_dim(thetas)
        for i in range(len(thetas)):
            p.setJointMotorControl2(
                bodyUniqueId=self._robot_id, jointIndex=i,
                controlMode=p.POSITION_CONTROL, targetPosition=thetas[i]
            )

    def set_jump_joint(self, thetas):
        self._chk_thetas_dim(thetas)
        for i in range(thetas.shape[0]):
            p.resetJointState(bodyUniqueId=self._robot_id, jointIndex=i, targetValue=thetas[i])

    def convert_pos_to_theta(self, pos, force=False):
        raise InterruptedError("convert_pos_to_theta() is necessary override.")

    def run_gripper(self, open=False, close=False):
        if self._hand is None:
            return
        self._hand.run(open, close)


class PyBullet3DoFRobot(PyBulletRobot):
    _DIM_JOINT = DIMENTION_3D
    _DIM_POSE  = DIMENTION_3D

    def __init__(self, robot_id, interpolation, hand):
        super().__init__(robot_id, interpolation, hand)

    def __inverse_kinematics(self, pos):
        self._chk_pos_dim(pos)
        thetas = p.calculateInverseKinematics(self._robot_id, self._n_joints, pos)
        return np.array(thetas)

    def convert_pos_to_theta(self, pos, force=False):
        if force:
            thetas = self.__inverse_kinematics(pos)
        else:
            if self._interpolation == INTERPOLATION.POSITION.value:
                thetas = self.__inverse_kinematics(pos)
            else:
                thetas = np.copy(pos)
        return thetas


class MainPyBulletRobot:
    _PLANE_URDF     = "plane.urdf"
    _SIMULATION_SLEEP_TIME = 0.05
    _INTERFERENCE_MARGIN   = 0.15
    _PATH_PLAN_TIME = 100
    _N_MARGIN_MOVE  = 50

    _GRASP_OBJECT_POS_3DOF    = [ 1.25,  0.4, 0.55]
    _GRASP_OBJECT_OFFSET_3DOF = [-0.15,  0,   0.15]
    _ENVIRONMENT_POS_3DOF     = [ 1.5,   0,  -0.5 ]


    def __init__(self, interpolation, n_robot_joint, environment_urdf=None, grasp_urdf=None, hand=False):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -GRABITY_VALUE)

        self._init_robot(n_robot_joint, interpolation, hand)
        self._init_environment(environment_urdf, grasp_urdf)
        self._rrt = None

    def _init_robot(self, n_robot_joint, interpolation, hand):
        if n_robot_joint == DIMENTION_3D:
            robot_urdf = ROBOTURDF.DOF3_HAND.value if hand else ROBOTURDF.DOF3.value
            robot_cls  = PyBullet3DoFRobot
        else:
            raise ValueError(f"n_robot_joint is abnormal. n_robot_joint is {n_robot_joint}.")

        self._robot_id = p.loadURDF(robot_urdf, basePosition=[0, 0, 0], useFixedBase=True)
        self._robot    = robot_cls(self._robot_id, interpolation, hand)

        self._interpolation = interpolation
        self._n_robot_joint = n_robot_joint

    def _init_environment(self, environment_urdf, grasp_urdf):
        p.loadURDF(self._PLANE_URDF)

        self._environment_id = None
        if environment_urdf is not None:
            basePosition = self._ENVIRONMENT_POS_3DOF
            self._environment_id = p.loadURDF(environment_urdf, basePosition=basePosition, useFixedBase=True)

        self._grasp_id           = None
        self._grasp_constraint_id = None
        if grasp_urdf is not None:
            self._grasp_id = p.loadURDF(grasp_urdf, basePosition=self._GRASP_OBJECT_POS_3DOF)
            p.changeDynamics(self._grasp_id, -1, lateralFriction=1.0, spinningFriction=1.0, rollingFriction=1)

            grasp_pos, grasp_ori = self._get_grasp_pos(offset=[0, 0, 0])
            self._grasp_constraint_id = self._set_constraint(self._grasp_id, grasp_pos, grasp_ori)


    def run(self, start_pos, path_plan):
        p.setRealTimeSimulation(1)

        offset  = self._GRASP_OBJECT_OFFSET_3DOF
        end_pos, _ = self._get_grasp_pos(offset)
        end_pos = np.array(end_pos)

        self._is_interference_start_end_pos(start_pos, end_pos)

        if path_plan == PATHPLAN.RRT.value:
            self._rrt = RRTPyBullet()
        else:
            raise ValueError(f"path_plan is abnormal. path_plan is {path_plan}")

        result = self._path_planning(start_pos, end_pos)

        if result:
            self._post_path_planning(start_pos, end_pos)

        self._rrt.save()
        self._exec_margin_time(end_pos, np.array(offset))

        return result

    def _path_planning(self, start_pos, end_pos):
        result = False
        self._rrt.preparation(start_pos, end_pos, self._interpolation)

        start_time = time.time()
        while True:
            if (time.time() - start_time) >= self._PATH_PLAN_TIME:
                break

            new_node_pos, near_node_pos, near_node = self._rrt.expand_once(end_pos)
            if self._is_line_interference(new_node_pos, near_node_pos):
                continue
            if not self._rrt.add_node_and_chk_goal(end_pos, new_node_pos, near_node):
                continue
            if not self._is_line_interference(new_node_pos, end_pos):
                result = True
                break

        return result

    def _post_path_planning(self, start_pos, end_pos):
        self._rrt.fin_planning(start_pos, end_pos)

        theta = self._robot.convert_pos_to_theta(start_pos)
        self._robot.set_jump_joint(theta)
        self._robot.run_gripper(open=True)

        for row_idx in range(self._rrt.pathes.shape[0]):
            next_theta = self._robot.convert_pos_to_theta(self._rrt.pathes[row_idx])
            self._robot.set_joint(next_theta)
            self._robot.run_gripper(open=True)
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _exec_margin_time(self, end_pos, offset):
        end_pos   = end_pos - offset
        end_theta = self._robot.convert_pos_to_theta(end_pos)

        for _ in range(self._N_MARGIN_MOVE):
            self._robot.set_joint(end_theta)
            self._robot.run_gripper(open=True)
            time.sleep(self._SIMULATION_SLEEP_TIME)

        if self._grasp_constraint_id is not None:
            p.removeConstraint(self._grasp_constraint_id)
            time.sleep(self._SIMULATION_SLEEP_TIME)
            self._grasp_constraint_id = None

        for _ in range(self._N_MARGIN_MOVE):
            self._robot.set_joint(end_theta)
            self._robot.run_gripper(close=True)
            time.sleep(self._SIMULATION_SLEEP_TIME)

    def _set_constraint(self, object_id, pos, ori):
        return p.createConstraint(
            object_id, -1, -1, -1,
            p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], pos,
            parentFrameOrientation=ori,
            childFrameOrientation=[0, 0, 0, 1]
        )

    def _get_grasp_pos(self, offset=None, dim2=False):
        if self._grasp_id is None:
            raise ValueError("self._grasp_id is None.")

        grasp_pos, grasp_ori = p.getBasePositionAndOrientation(self._grasp_id)

        if offset is not None:
            grasp_pos_offset = [pos + off for pos, off in zip(grasp_pos, offset)]
        else:
            grasp_pos_offset = list(grasp_pos)

        if dim2:
            grasp_pos_offset = grasp_pos_offset[:DIMENTION_2D]

        return grasp_pos_offset, grasp_ori

    def _is_line_interference(self, pos1, pos2):
        is_interference = True
        if self._is_interference_pos(pos2):
            return is_interference
        if self._is_interference_pos(pos1):
            return is_interference

        theta = self._robot.convert_pos_to_theta(pos2)
        self._robot.set_joint(theta)
        self._robot.run_gripper(open=True)
        time.sleep(self._SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference

    def _is_interference_start_end_pos(self, start_pos, end_pos):
        if self._is_interference_pos(start_pos):
            raise ValueError("start_pos is interference. change start_pos.")
        if self._is_interference_pos(end_pos):
            raise ValueError("end_pos is interference. change end_pos.")

    def _is_interference_pos(self, pos):
        is_interference = True
        theta = self._robot.convert_pos_to_theta(pos)
        self._robot.set_jump_joint(theta)
        self._robot.run_gripper(open=True)
        time.sleep(self._SIMULATION_SLEEP_TIME)

        close_points = p.getClosestPoints(self._robot_id, self._environment_id, self._INTERFERENCE_MARGIN)
        if len(close_points) == 0:
            is_interference = False

        return is_interference

    def convert_pos_to_theta(self, pos):
        return self._robot.convert_pos_to_theta(pos, force=True)
