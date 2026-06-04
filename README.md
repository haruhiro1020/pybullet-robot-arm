# PyBulletによるロボットアームシミュレーション — ソースコード

PyBullet（Python向け物理シミュレータ）を使ってロボットアームを動かすシミュレーションのソースコードです。
2軸ロボットアームを対象に、関節角度制御・逆運動学・経路計画（RRT / RRT-Connect / RRT*）・物体把持・カメラ認識まで段階的に実装しています。

---

## ディレクトリ構成

```
src/
└── two-dof/                   # 2軸ロボットアーム（7パート）
    ├── part1_joint/           # 関節角度制御
    ├── part2_position/        # 逆運動学による手先位置制御
    ├── part3_rrt/             # RRTによる衝突回避経路計画
    ├── part4_grasp/           # グリッパーによる物体把持
    ├── part5_rrt_grasp/       # RRTを用いた物体把持の経路計画
    ├── part6_single_camera/   # 単眼カメラによる物体位置推定と把持
    └── part7_multi_camera/    # 多眼カメラによるロバストな物体把持
```

---

## 2軸ロボットアーム (two-dof)

| パート | 内容 |
|---|---|
| [part1_joint](two-dof/part1_joint/) | 関節角度制御 |
| [part2_position](two-dof/part2_position/) | 逆運動学による手先位置制御 |
| [part3_rrt](two-dof/part3_rrt/) | RRTによる衝突回避経路計画 |
| [part4_grasp](two-dof/part4_grasp/) | グリッパーによる物体把持 |
| [part5_rrt_grasp](two-dof/part5_rrt_grasp/) | RRTを用いた物体把持の経路計画 |
| [part6_single_camera](two-dof/part6_single_camera/) | 単眼カメラによる物体位置推定と把持 |
| [part7_multi_camera](two-dof/part7_multi_camera/) | 多眼カメラによるロバストな物体把持 |

---


## 動作環境

| ソフトウェア | バージョン |
|---|---|
| OS | macOS Sequoia 15.5 |
| Python | 3.13.3 |
| PyBullet | 3.2.5 |
| NumPy | 2.3.0 |

---

## 実行方法

各パートの `main.py` を実行してください。

```bash
# 例：2軸ロボットアームの関節角度制御
cd two-dof/part1_joint
python main.py
```

PyBulletのGUIウィンドウが開き、スライダーや自動経路計画でロボットを動かすことができます。

---

## ファイル構成の説明

各パートで共通して使われるファイルの役割を下表に示します。

| ファイル名 | 役割 |
|---|---|
| `constant.py` | 全ファイル共通の定数・Enum定義 |
| `main.py` | エントリーポイント（パラメータ設定・実行） |
| `pybullet_robot.py` | ロボットの読み込み・制御・逆運動学 |
| `pybullet_rrt.py` | RRT / RRT-Connect / RRT* による経路計画 |
| `pybullet_environment.py` | 干渉物環境の読み込み・管理 |
| `pybullet_grasp.py` | 把持対象物の読み込み・拘束・位置取得 |
| `pybullet_gripper.py` | パラレルグリッパーの開閉制御 |
| `pybullet_interference.py` | ロボットと環境の干渉判定 |
| `pybullet_camera.py` | カメラによる物体位置推定 |
| `pybullet_main.py` | PyBulletの統合メイン処理 |
| `robot_Xdof.urdf` | グリッパーなしロボットアームのモデル定義 |
| `robot_Xdof_hand.urdf` | グリッパー付きロボットアームのモデル定義 |
| `environment.urdf` | 干渉物（障害物）の環境モデル定義 |
| `grasp_object.urdf` | 把持対象物（立方体）のモデル定義 |
| `camera_*.urdf` | 各方向を向いたカメラのモデル定義 |
