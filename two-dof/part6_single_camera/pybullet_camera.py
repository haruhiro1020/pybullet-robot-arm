# PyBulletで使用するカメラを記載


# ライブラリの読み込み
import pybullet as p    # PyBullet
import numpy as np      # 数値計算ライブラリ


# 自作モジュールの読み込み
from constant import *



class PyBulletCamera:
    # 定数の定義
    _FOV    = 90        # 視野角 ([degree])
    _ASPECT = 1.0       # 縦横比
    _NEAR   = 0.01      # 最近接面
    _FAR    = 10.0      # 最遠面
    _WIDTH  = 320       # 画像の横幅 [pixel]
    _HEIGHT = 240       # 画像の縦幅 [pixel]


    def __init__(self, urdf, base_pos=[0, 0, 3]):
        """
        コンストラクタ
        
        パラメータ
            urdf(str): カメラのURDFファイル名
        """
        # カメラを読み込む
        self.__camera_id = p.loadURDF(urdf, basePosition=base_pos, useFixedBase=True)

        # カメラ情報を取得
        camera_infos = p.getLinkState(self.__camera_id, 0)

        # カメラの位置を取得
        self.__camera_pos = camera_infos[0]

        # 画像の上側 ([0, 1, 0]としたら，上手く動いた)
        camera_up_vector = [0, 1, 0]

        # カメラ位置や方向の指定
        self.__view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.__camera_pos,       # カメラ位置
            cameraTargetPosition=[0, 0, 0],     # カメラの目標点
            cameraUpVector=camera_up_vector     # 画面の上側ベクトル
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
        位置を取得

        パラメータ
            target_id(int): カメラで取得したい目標物のID番号 (ID番号はPyBulletのloadURDF()の戻り値)

        戻り値
            numpy.ndarray: 目標物の位置
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
            print(f"target {target_id} is not visible in camera")
            raise ValueError(f"target {target_id} is not visible in camera")

        # 物体の中心ピクセルを取得
        x_center = np.mean(xs)
        y_center = np.mean(ys)

        # 物体に向かって光線を照射して，位置を取得
        pos = self.__get_one_ray_result(x_center, y_center)

        return pos


    def __get_one_ray_result(self, x_pixel, y_pixel):
        """
        光線を照射して，物体位置の取得

        パラメータ
            x_pixel(float): 光線のX方向位置 [pixel]
            y_pixel(float): 光線のY方向位置 [pixel]

        戻り値
            numpy.ndarray: 目標物の位置
        """
        # 事前準備
        inv_view_matrix = np.linalg.inv(np.array(self.__view_matrix).reshape(4, 4))
        inv_project_matrix = np.linalg.inv(np.array(self.__projection_matrix).reshape(4, 4))

        # 画像座標(ピクセル) を 正規化デバイス座標(NDC)へ変換する
        x_ndc = 2 * x_pixel / self._WIDTH - 1
        y_ndc = 1 - 2 * y_pixel / self._HEIGHT

        # クリップ座標
        clip_coords = np.array([x_ndc, y_ndc, -1.0, 1.0])

        # カメラ
        eye_coords = np.dot(inv_project_matrix, clip_coords)
        eye_coords = np.array([eye_coords[0], eye_coords[1], -1.0, 0.0])

        # 光線の方向
        ray_dir = np.dot(inv_view_matrix, eye_coords)
        ray_dir = ray_dir[:3]

        # 光線の照射先の位置
        ray_to = self.__camera_pos + ray_dir * 10

        # 光線結果
        # rayTest()の引数は以下の通り
        # 第１引数：光線の照射元の位置
        # 第２引数：光線の照射先の位置
        # rayTest()の返り値は以下の通り
        # 第１要素：光線が当たった物体のID(物体に当たらなかったら，-1)
        # 第２要素：光線が当たった物体のリンクの要素番号(リンクがない場合は，-1)
        # 第３要素：光線の照射元から照射先までの距離に対する，当たった位置の割合(0.0 ... 照射元の位置，1.0 ... 照射先の位置)
        # 第４要素：光線が当たった物体の位置
        # 第５要素：光線が当たった物体の法線ベクトル
        result = p.rayTest(self.__camera_pos, ray_to)
        pos = np.array(result[0][3])

        return pos
