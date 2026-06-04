# PyBulletで使用するグリッパーを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *



class _PyBulletCamera:
    # 定数の定義
    _FOV    = 90        # 視野角 ([degree])
    _ASPECT = 1.0       # 縦横比
    _NEAR   = 0.01      # 最近接面
    _FAR    = 10.0      # 最遠面
    _WIDTH  = 320       # 画像の横幅 [pixel]
    _HEIGHT = 240       # 画像の縦幅 [pixel]


    def __init__(self, urdf, base_pos=[0, 0, 5], target_pos=[0, 0, 0], camera_up_vector=[0, 1, 0]):
        """
        コンストラクタ

        パラメータ
            urdf(str): カメラのURDFファイル名
            base_pos(list): カメラ位置
            target_pos(list): カメラが向いている位置
            camera_up_vector(list): カメラの上方向ベクトル
        """
        # カメラを読み込む
        self.__camera_id = p.loadURDF(urdf, basePosition=base_pos, useFixedBase=True)

        # カメラ情報を取得
        camera_infos = p.getLinkState(self.__camera_id, 0)

        # カメラの位置を取得
        self.__camera_pos = camera_infos[0]

        # カメラ位置や方向の指定
        self.__view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.__camera_pos,    # カメラ位置
            cameraTargetPosition=target_pos,        # カメラの目標点
            cameraUpVector=camera_up_vector         # 画面の上側ベクトル
        )

        # カメラの3D情報を2Dへ変換するための情報設定
        self.__projection_matrix = p.computeProjectionMatrixFOV(
            fov=self._FOV,              # 視野角 [degree]
            aspect=self._ASPECT,        # アスペクト比
            nearVal=self._NEAR,         # 最近距離
            farVal=self._FAR            # 最遠距離
        )


    def get_pos(self, target_id):
        """
        目標物の位置を取得

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 目標物の位置と目標物を検出できたピクセル数を取得
        pos, n_pixel = self.get_pos_and_pixelnum(target_id)

        return pos


    def get_pos_and_pixelnum(self, target_id):
        """
        目標物の位置と目標物を検出できたピクセル数を取得

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
            int: 目標物が検出できたピクセル数
        """
        # カメラ画像の取得
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=self._WIDTH,
            height=self._HEIGHT,
            viewMatrix=self.__view_matrix,
            projectionMatrix=self.__projection_matrix
        )

        # Numpy型への型変換
        segImg = np.array(segImg)
        # 目標物体が描画されている画像の抽出
        ys, xs = np.where(segImg == target_id)
        if len(xs) == 0 or len(ys) == 0:
            # 目標物体が描画されていないため，エラー発行
            raise ValueError(f"target {target_id} is not visible in camera")

        # 物体の中心ピクセルを取得
        x_center = np.mean(xs)
        y_center = np.mean(ys)
        print(f"x_center = {x_center}")
        print(f"y_center = {y_center}")
        print(f"len(xs) = {len(xs)}")

        # 物体に向かって光線を照射して，位置を取得
        pos = self.__get_ray_result(x_center, y_center, target_id)

        return pos, len(xs)


    def __get_ray_result(self, x_pixel, y_pixel, target_id):
        """
        光線を照射して，目標物体の位置を取得

        パラメータ
            x_pixel(float): 光線のX方向位置 [pixel]
            y_pixel(float): 光線のY方向位置 [pixel]
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 事前準備
        inv_view_matrix    = np.linalg.inv(np.array(self.__view_matrix).reshape(4, 4))
        inv_project_matrix = np.linalg.inv(np.array(self.__projection_matrix).reshape(4, 4))

        # 画像座標(ピクセル) を 正規化デバイス座標(NDC)へ変換する
        x_ndc = 2 * x_pixel / self._WIDTH  - 1
        y_ndc = 1 - 2 * y_pixel / self._HEIGHT

        # クリップ座標
        clip_coords = np.array([x_ndc, y_ndc, -1.0, 1.0])

        # カメラ
        eye_coords = np.dot(inv_project_matrix, clip_coords)
        eye_coords = np.array([eye_coords[0], eye_coords[1], -1.0, 0.0])

        # 光線の方向
        ray_dir = np.dot(inv_view_matrix, eye_coords)
        ray_dir = ray_dir[:3] / np.linalg.norm(ray_dir[:3])

        # 光線の照射先の位置
        ray_to = self.__camera_pos + ray_dir * self._FAR

        # 光線結果
        # rayTest()の引数は以下の通り
        # 第１引数：光線の照射元の位置
        # 第２引数：光線の照射先の位置
        # rayTest()の返り値は以下の通り
        # 第１要素：物体ID
        # 第２要素：？
        # 第３要素：？
        # 第４要素：位置
        # 第５要素：？
        result = p.rayTest(self.__camera_pos, ray_to)
        print(f"self.__camera_pos = {self.__camera_pos}")
        print(f"ray_to = {ray_to}")

        ray_id = result[0][0]
        if ray_id != target_id:
            # 求めたい物体IDと光線で取得した物体IDが異なると，光線による物体位置が求めたい物体位置とは異なる
            print(f"ray_id and target_id are not matched. ray_id = {ray_id}. target_id = {target_id}")
            raise ValueError(f"ray_id and target_id are not matched. ray_id = {ray_id}. target_id = {target_id}")

        pos = np.array(result[0][3])

        return pos



class _PyBulletMultiCamera:
    # 定数の定義

    # カメラ位置に関する定数
    __RIGHT2LEFT_POS = [ 5,  0, 1]      # 左向き(-X方向)のカメラ位置
    __LEFT2RIGHT_POS = [-5,  0, 1]      # 右向き(+X方向)のカメラ位置
    __FRONT2BACK_POS = [ 0, -5, 1]      # 奥向き(+Y方向)のカメラ位置
    __BACK2FRONT_POS = [ 0,  5, 1]      # 手前向き(-Y方向)のカメラ位置
    __UP2DOWN_POS    = [ 0,  0, 5]      # 下向き(-Z方向)のカメラ位置

    # カメラの上側に関する定数
    __RIGHT2LEFT_UP_VECTOR = [0, 0, 1]  # 左向きカメラの上側ベクトル
    __LEFT2RIGHT_UP_VECTOR = [0, 0, 1]  # 右向きカメラの上側ベクトル
    __FRONT2BACK_UP_VECTOR = [0, 0, 1]  # 奥向きカメラの上側ベクトル
    __BACK2FRONT_UP_VECTOR = [0, 0, 1]  # 手前向きカメラの上側ベクトル
    __UP2DOWN_UP_VECTOR    = [0, 1, 0]  # 下向きカメラの上側ベクトル

    # カメラの向いている位置に関する定数
    __RIGHT2LEFT_TARGET_POS = [0                  , __RIGHT2LEFT_POS[1], __RIGHT2LEFT_POS[2]]   # 左向きカメラが向いている位置
    __LEFT2RIGHT_TARGET_POS = [0                  , __LEFT2RIGHT_POS[1], __LEFT2RIGHT_POS[2]]   # 右向きカメラが向いている位置
    __FRONT2BACK_TARGET_POS = [__FRONT2BACK_POS[0], 0                  , __FRONT2BACK_POS[2]]   # 奥向きカメラが向いている位置
    __BACK2FRONT_TARGET_POS = [__BACK2FRONT_POS[0], 0                  , __BACK2FRONT_POS[2]]   # 手前向きカメラが向いている位置
    __UP2DOWN_TARGET_POS    = [__UP2DOWN_POS[0]   , __UP2DOWN_POS[1]   , 0                  ]   # 手前向きカメラが向いている位置


    def __init__(self, n_cameras):
        """
        コンストラクタ

        パラメータ
            n_cameras(CAMERANUM): カメラ数
        """
        # 今回はカメラ台数を 5 の固定数とする
        if n_cameras != CAMERANUM.MULTI.value:
            raise ValueError(f"n_cameras is abnormal. n_camera is {n_cameras}")

        # 5台のカメラ作成
        self.__cameras = []
        self.__make_multi_cameras()


    def __make_multi_cameras(self):
        """
        複数のカメラを作成
        """
        # 下向きのカメラを作成
        self.__cameras.append(_PyBulletCamera(
                            CAMERAURDF.UP2DOWN.value,
                            self.__UP2DOWN_POS,
                            self.__UP2DOWN_TARGET_POS,
                            self.__UP2DOWN_UP_VECTOR))

        # 左向きのカメラを作成
        self.__cameras.append(_PyBulletCamera(
                            CAMERAURDF.RIGHT2LEFT.value,
                            self.__RIGHT2LEFT_POS,
                            self.__RIGHT2LEFT_TARGET_POS,
                            self.__RIGHT2LEFT_UP_VECTOR))
        # 右向きのカメラを作成
        self.__cameras.append(_PyBulletCamera(
                            CAMERAURDF.LEFT2RIGHT.value,
                            self.__LEFT2RIGHT_POS,
                            self.__LEFT2RIGHT_TARGET_POS,
                            self.__LEFT2RIGHT_UP_VECTOR))

        # 奥向きのカメラを作成
        self.__cameras.append(_PyBulletCamera(
                            CAMERAURDF.BACK2FRONT.value,
                            self.__BACK2FRONT_POS,
                            self.__BACK2FRONT_TARGET_POS,
                            self.__BACK2FRONT_UP_VECTOR))
        # 手前向きのカメラを作成
        self.__cameras.append(_PyBulletCamera(
                            CAMERAURDF.FRONT2BACK.value,
                            self.__FRONT2BACK_POS,
                            self.__FRONT2BACK_TARGET_POS,
                            self.__FRONT2BACK_UP_VECTOR))


    def get_pos(self, target_id):
        """
        目標物の位置を取得

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 全カメラを使って，目標物の位置を取得する
        target_pos = self.__get_all_cameras_pos(target_id)

        return target_pos


    def __get_all_cameras_pos(self, target_id):
        """
        全カメラより，目標物の位置を取得する

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 全カメラから取得した，目標物の位置を保存
        all_cameras_target_pos = []
        # 全カメラから取得した，目標物を検出できたピクセル数を保存
        all_cameras_pixel_num  = []

        # 全カメラ分，ループする
        for idx, camera in enumerate(self.__cameras):

            try:
                # カメラから目標物の位置を取得する
                target_pos, n_pixel = camera.get_pos_and_pixelnum(target_id)
                # 目標物の位置を保存
                all_cameras_target_pos.append(target_pos)
                # 目標物を検出できたピクセル数を保存
                all_cameras_pixel_num.append(n_pixel)
                print(f"{idx} camera search target. target is exist.")
            except Exception as e:
                # カメラから目標物の位置を取得できなかった
                print(f"{idx} camera doesn't search target. target is not exist.")

            print("")

        if len(all_cameras_target_pos) == 0 or len(all_cameras_pixel_num) == 0:
            # 全カメラで目標物の位置を取得できなかった
            raise ValueError("len(all_cameras_target_pos) == 0 or len(all_cameras_pixel_num) == 0")

        if len(all_cameras_target_pos) != len(all_cameras_pixel_num):
            # データ数が不一致で異常
            raise ValueError(f"len(all_cameras_target_pos) and len(all_cameras_pixel_num) are not matched. len(all_cameras_target_pos) = {len(all_cameras_target_pos)}. len(all_cameras_pixel_num) = {len(all_cameras_pixel_num)}")

        # 重み付き平均による目標物の位置を計算
        target_pos = self.__calc_weighted_target_pos(all_cameras_target_pos, all_cameras_pixel_num)

        return target_pos


    def __calc_weighted_target_pos(self, all_cameras_pos, all_cameras_pixels):
        """
        重み付き平均による目標物の位置を計算

        パラメータ
            all_cameras_pos(list): 全カメラで取得した目標物の位置
            all_cameras_pixels(list): 全カメラで取得した目標物のピクセル数

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 引数の確認
        if len(all_cameras_pos) == 0 or len(all_cameras_pixels) == 0:
            # データが存在しないため，異常
            raise ValueError("len(all_cameras_pos) == 0 or len(all_cameras_pixels) == 0")

        if len(all_cameras_pos) != len(all_cameras_pixels):
            # データ数が不一致で異常
            raise ValueError(f"len(all_cameras_pos) and len(all_cameras_pixels) are not matched. len(all_cameras_pos) = {len(all_cameras_pos)}. len(all_cameras_pixels) = {len(all_cameras_pixels)}")

        # list型からnumpy.ndarray型に型変換 (行列計算による処理速度の向上のために型変換)
        all_cameras_pos_np = np.array(all_cameras_pos)
        all_cameras_pixels_np = np.array(all_cameras_pixels)

        # 目標物の位置をピクセル数による重み付き平均によって，計算する
        target_pos = np.dot(all_cameras_pixels_np, all_cameras_pos_np) / np.sum(all_cameras_pixels_np)

        return target_pos


class PyBulletCameraContoller:
    # 定数の定義


    def __init__(self, n_cameras):
        """
        コンストラクタ

        パラメータ
            n_cameras(CAMERANUM): カメラ数
        """
        # 引数の確認
        if n_cameras == CAMERANUM.SINGLE.value:     # 1つのカメラ
            # 上から下に向いてる
            self.__camera = _PyBulletCamera(CAMERAURDF.UP2DOWN.value)
        elif n_cameras == CAMERANUM.MULTI.value:    # 複数のカメラ
            self.__camera = _PyBulletMultiCamera(n_cameras)
        else:                                       # 異常な数
            raise ValueError(f"n_cameras is abnormal. n_cameras is {n_cameras}")


    def get_pos(self, target_id):
        """
        目標物の位置を取得

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
        """
        return self.__camera.get_pos(target_id)
