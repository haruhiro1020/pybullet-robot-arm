import pybullet as p
import numpy as np

from constant import *


class _PyBulletCamera:
    _FOV    = 90
    _ASPECT = 1.0
    _NEAR   = 0.01
    _FAR    = 10.0
    _WIDTH  = 320
    _HEIGHT = 240


    def __init__(self, urdf, base_pos=[0, 0, 5], target_pos=[0, 0, 0], camera_up_vector=[0, 1, 0]):
        self.__camera_id = p.loadURDF(urdf, basePosition=base_pos, useFixedBase=True)

        camera_infos      = p.getLinkState(self.__camera_id, 0)
        self.__camera_pos = camera_infos[0]

        self.__view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.__camera_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=camera_up_vector
        )

        self.__projection_matrix = p.computeProjectionMatrixFOV(
            fov=self._FOV,
            aspect=self._ASPECT,
            nearVal=self._NEAR,
            farVal=self._FAR
        )

    def get_pos(self, target_id):
        pos, _ = self.get_pos_and_pixelnum(target_id)
        return pos

    def get_pos_and_pixelnum(self, target_id):
        _, _, _, _, segImg = p.getCameraImage(
            width=self._WIDTH,
            height=self._HEIGHT,
            viewMatrix=self.__view_matrix,
            projectionMatrix=self.__projection_matrix
        )

        segImg = np.array(segImg)
        ys, xs = np.where(segImg == target_id)
        if len(xs) == 0 or len(ys) == 0:
            raise ValueError(f"target {target_id} is not visible in camera")

        x_center = np.mean(xs)
        y_center = np.mean(ys)

        pos = self.__get_ray_result(x_center, y_center, target_id)

        return pos, len(xs)

    def __get_ray_result(self, x_pixel, y_pixel, target_id):
        inv_view_matrix    = np.linalg.inv(np.array(self.__view_matrix).reshape(4, 4))
        inv_project_matrix = np.linalg.inv(np.array(self.__projection_matrix).reshape(4, 4))

        x_ndc = 2 * x_pixel / self._WIDTH - 1
        y_ndc = 1 - 2 * y_pixel / self._HEIGHT

        clip_coords = np.array([x_ndc, y_ndc, -1.0, 1.0])

        eye_coords = np.dot(inv_project_matrix, clip_coords)
        eye_coords = np.array([eye_coords[0], eye_coords[1], -1.0, 0.0])

        ray_dir = np.dot(inv_view_matrix, eye_coords)
        ray_dir = ray_dir[:3] / np.linalg.norm(ray_dir[:3])

        ray_to = self.__camera_pos + ray_dir * self._FAR

        result = p.rayTest(self.__camera_pos, ray_to)

        ray_id = result[0][0]
        if ray_id != target_id:
            raise ValueError(f"ray_id and target_id are not matched. ray_id={ray_id}, target_id={target_id}")

        return np.array(result[0][3])


class _PyBulletMultiCamera:
    __RIGHT2LEFT_POS = [ 5,  0, 1]
    __LEFT2RIGHT_POS = [-5,  0, 1]
    __FRONT2BACK_POS = [ 0, -5, 1]
    __BACK2FRONT_POS = [ 0,  5, 1]
    __UP2DOWN_POS    = [ 0,  0, 5]

    __RIGHT2LEFT_UP_VECTOR = [0, 0, 1]
    __LEFT2RIGHT_UP_VECTOR = [0, 0, 1]
    __FRONT2BACK_UP_VECTOR = [0, 0, 1]
    __BACK2FRONT_UP_VECTOR = [0, 0, 1]
    __UP2DOWN_UP_VECTOR    = [0, 1, 0]

    __RIGHT2LEFT_TARGET_POS = [0                  , __RIGHT2LEFT_POS[1], __RIGHT2LEFT_POS[2]]
    __LEFT2RIGHT_TARGET_POS = [0                  , __LEFT2RIGHT_POS[1], __LEFT2RIGHT_POS[2]]
    __FRONT2BACK_TARGET_POS = [__FRONT2BACK_POS[0], 0                  , __FRONT2BACK_POS[2]]
    __BACK2FRONT_TARGET_POS = [__BACK2FRONT_POS[0], 0                  , __BACK2FRONT_POS[2]]
    __UP2DOWN_TARGET_POS    = [__UP2DOWN_POS[0]   , __UP2DOWN_POS[1]   , 0                  ]


    def __init__(self, n_cameras):
        if n_cameras != CAMERANUM.MULTI.value:
            raise ValueError(f"n_cameras is abnormal. n_cameras is {n_cameras}")

        self.__cameras = []
        self.__make_multi_cameras()

    def __make_multi_cameras(self):
        self.__cameras.append(_PyBulletCamera(
            CAMERAURDF.UP2DOWN.value,
            self.__UP2DOWN_POS, self.__UP2DOWN_TARGET_POS, self.__UP2DOWN_UP_VECTOR))

        self.__cameras.append(_PyBulletCamera(
            CAMERAURDF.RIGHT2LEFT.value,
            self.__RIGHT2LEFT_POS, self.__RIGHT2LEFT_TARGET_POS, self.__RIGHT2LEFT_UP_VECTOR))

        self.__cameras.append(_PyBulletCamera(
            CAMERAURDF.LEFT2RIGHT.value,
            self.__LEFT2RIGHT_POS, self.__LEFT2RIGHT_TARGET_POS, self.__LEFT2RIGHT_UP_VECTOR))

        self.__cameras.append(_PyBulletCamera(
            CAMERAURDF.BACK2FRONT.value,
            self.__BACK2FRONT_POS, self.__BACK2FRONT_TARGET_POS, self.__BACK2FRONT_UP_VECTOR))

        self.__cameras.append(_PyBulletCamera(
            CAMERAURDF.FRONT2BACK.value,
            self.__FRONT2BACK_POS, self.__FRONT2BACK_TARGET_POS, self.__FRONT2BACK_UP_VECTOR))

    def get_pos(self, target_id):
        return self.__get_all_cameras_pos(target_id)

    def __get_all_cameras_pos(self, target_id):
        all_cameras_target_pos = []
        all_cameras_pixel_num  = []

        for idx, camera in enumerate(self.__cameras):
            try:
                target_pos, n_pixel = camera.get_pos_and_pixelnum(target_id)
                all_cameras_target_pos.append(target_pos)
                all_cameras_pixel_num.append(n_pixel)
            except Exception:
                print(f"camera {idx}: target not visible")

        if len(all_cameras_target_pos) == 0:
            raise ValueError("No camera could detect the target.")

        return self.__calc_weighted_target_pos(all_cameras_target_pos, all_cameras_pixel_num)

    def __calc_weighted_target_pos(self, all_cameras_pos, all_cameras_pixels):
        if len(all_cameras_pos) == 0:
            raise ValueError("all_cameras_pos is empty.")

        pos_np    = np.array(all_cameras_pos)
        pixels_np = np.array(all_cameras_pixels)

        return np.dot(pixels_np, pos_np) / np.sum(pixels_np)


class PyBulletCameraContoller:
    def __init__(self, n_cameras):
        if n_cameras == CAMERANUM.SINGLE.value:
            self.__camera = _PyBulletCamera(CAMERAURDF.UP2DOWN.value)
        elif n_cameras == CAMERANUM.MULTI.value:
            self.__camera = _PyBulletMultiCamera(n_cameras)
        else:
            raise ValueError(f"n_cameras is abnormal. n_cameras is {n_cameras}")

    def get_pos(self, target_id):
        return self.__camera.get_pos(target_id)
