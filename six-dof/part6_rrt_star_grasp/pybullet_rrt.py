# 経路生成手法であるRRT (Rapidly-exploring Random Tree) の実装 (PyBullet用)


# ライブラリの読み込み
import numpy as np
import os
import time


# 自作モジュールの読み込み
from constant import *              # 定数
from pybullet_interference import PyBulletInteference   # 干渉判定に関して



class _Tree:
    def __init__(self, near_node_idx):
        """
        コンストラクタ
        """
        # プロパティの初期化
        self._nodes = []
        self._near_node_idx = near_node_idx

    @property
    def nodes(self):
        """
        _nodesプロパティのゲッター
        """
        return self._nodes

    def reset(self):
        """
        データの初期化
        """
        if len(self._nodes) != 0:
            # 何かしらのデータが保存
            del self._nodes
        self._nodes = []

    def add_node(self, node):
        """
        ノードの追加

        パラメータ
            node(numpy.ndarray): ノード
        """
        if len(self._nodes) == 0:       # 初回だけ実行
            self._nodes = node
        else:
            self._nodes = np.append(self._nodes, node, axis=0)

    def _chk_node_exist(self):
        """
        ノードが存在するかの確認
        """
        if len(self._nodes) == 0:
            # 存在しない
            raise ValueError("self._nodes is not exist")

    def get_near_node(self, pos, weight):
        """
        最短距離のノードを取得

        パラメータ
            pos(numpy.ndarray): 位置
            weight(numpy.ndarray): 各次元の重み

        戻り値
            min_dist_idx(int): 最短距離のノード番号
        """
        # ノードの存在確認
        self._chk_node_exist()

        # ノードから位置を取得
        nodes_pos = self._nodes[:, :self._near_node_idx]
        # 差分を計算
        difference = nodes_pos - pos

        # 差分に重みを考慮
        if weight is None:
            weight = 1
        difference = difference * weight

        # 距離を計算 (各ノードとの距離を算出するため，引数にaxis=1を与えた)
        distance = np.linalg.norm(difference, axis=1)
        # 最短距離ノードを取得
        min_dist_idx = np.argmin(distance)

        return min_dist_idx

    def get_near_node_list(self, pos, radius):
        """
        ノードと近傍ノードをリストで取得

        パラメータ
            pos(numpy.ndarray): ノード位置
            radius(float): 半径

        戻り値
            near_node_list(list): 近傍ノードリスト
        """
        # ノードの存在確認
        self._chk_node_exist()

        near_node_list = []
        # ツリー内全ノード位置を取得
        all_node_pos = self._nodes[:, :self._near_node_idx]
        # ノードとツリー内全ノードの差分を計算
        difference = all_node_pos - pos
        # 差分から距離(ユークリッド距離)を計算
        distance = np.linalg.norm(difference, axis=1)

        # 距離が一定以内のノードだけを保存
        near_node_list = [idx for idx, dist in enumerate(distance) if dist <= radius]

        return near_node_list


class _CostedTree(_Tree):
    def __init__(self, near_node_idx, cost_idx):
        """
        コンストラクタ

        パラメータ
            near_node_idx(int): 最短ノード列番号
            cost_idx(int): コスト列番号
        """
        # プロパティの初期化
        self._nodes = []
        self._near_node_idx = near_node_idx
        self._cost_idx = cost_idx

    def calc_cost(self, node_idx, pos):
        """
        コストの計算

        パラメータ
            node_idx(int): ノード番号
            pos(numpy.ndarray): 位置

        戻り値
            cost(float): コスト
        """
        # ノードの存在確認
        self._chk_node_exist()

        # ノードから位置・コストを取得
        node = self._nodes[node_idx]
        node_pos  = node[:self._near_node_idx]
        node_cost = node[self._cost_idx]

        # 距離を計算
        distance = np.linalg.norm(node_pos - pos)
        # コストを計算
        cost = node_cost + distance

        return cost

    def chg_node_info(self, node_idx, near_node, cost):
        """
        ノード情報を変更

        パラメータ
            node_idx(int): 変更したいノード番号
            near_node(int): 変更後の最短ノード
            cost(float): 変更後のコスト
        """
        # ノードの存在確認
        self._chk_node_exist()

        self._nodes[node_idx, self._near_node_idx] = near_node
        self._nodes[node_idx, self._cost_idx] = cost


class _PyBulletRRT:
    # 定数の定義
    # 経路生成の名前
    _PATH_PLAN = PATHPLAN.RRT.value

    # ファイル名の定義
    # _pathesプロパティを保存するファイル名
    _FILE_NAME_PATHES = f"{_PATH_PLAN}_pathes.csv"
    # _start_treeプロパティを保存するファイル名
    _FILE_NAME_START_TREE = f"{_PATH_PLAN}_start_tree.csv"

    # ツリーの要素番号を定義
    _NODE_NEAR_NODE_IDX = RRT_NEAR_NODE_IDX     # ノード内の最短ノード要素

    # 探索に関する定義
    _MOVING_VALUE_JOINT = 0.1      # 1回の移動量 [rad] (ロボットの関節空間)
    _MOVING_VALUE_POS = 0.2         # 1回の移動量 [m] (ロボットの位置空間)
    _STRICT_PLANNING_ROB_JOINT = np.pi / 2  # 探索範囲の制限 [rad] (ロボットの関節空間)
    _STRICT_PLANNING_ROB_POS = 1.0  # 探索範囲の制限 [m] (ロボットの位置空間)

    _TIMEOUT_VALUE = 100        # タイムアウト時間 [second]
    _GOAL_SAMPLE_RATE = 0.1     # ランダムな値を取るときに，終点を選択する確率


    def __init__(self):
        """
        コンストラクタ
        """
        self._dim    = DIMENTION_NONE
        self._name   = self._PATH_PLAN
        self._pathes = []
        self._start_tree    = _Tree(self._NODE_NEAR_NODE_IDX)
        self._interpolation = INTERPOLATION.NONE.value
        self._moving_value  = self._MOVING_VALUE_JOINT


    @property
    def pathes(self):
        """
        _pathesプロパティのゲッター
        """
        return self._pathes


    def _preparation_planning(self, start_pos, end_pos, interpolation, interference:PyBulletInteference, joints_limit):
        """
        経路生成の準備

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            interpolation(INTERPOLATION): 補間方法 (関節空間/位置空間)
            interference(PyBulletInteference): 干渉判定クラス
            joints_limit(numpy.ndarray): 関節限界
        """
        # データの初期化
        self._reset()

        # 始点と終点の次元数が一致しているかの確認
        start_pos_dim = np.size(start_pos)
        end_pos_dim   = np.size(end_pos)
        if start_pos_dim != end_pos_dim:
            # 次元数が異なるので異常
            raise ValueError(f"start_pos_dim and end_pos_dim are not matched. start_pos_dim is {start_pos_dim}, end_pos_dim is {end_pos_dim}")

        # プロパティの更新
        self._dim = start_pos_dim
        self._interference = interference

        # 探索空間の設定
        self._set_interpolation(interpolation)

        # 始点と終点の干渉判定
        self._is_interference_start_end_pos(start_pos, end_pos)

        # 探索範囲を設定
        self._strict_planning_pos(start_pos, end_pos, joints_limit)

        # 結果を保存するフォルダ作成
        self._make_path_plan_folder(interpolation)

        # フォルダー内のファイルを全部削除
        self._reset_folder(interpolation)

    def _is_interference_start_end_pos(self, start_pos, end_pos):
        """
        始点と終点が干渉判定していないかの確認

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点 (直交空間/関節空間)
            end_pos(numpy.ndarray): 経路生成の終点 (直交空間/関節空間)
        """
        # 始点の干渉判定
        if self._interference.is_interference_pos(start_pos):
            raise ValueError("start_pos is interference. change start_pos.")

        # 終点の干渉判定
        if self._interference.is_interference_pos(end_pos):
            raise ValueError("end_pos is interference. change end_pos.")

    def planning(self, start_pos, end_pos, interpolation, interference, joints_limit, weight=None):
        """
        始点から終点までの経路生成の実行

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            interpolation(INTERPOLATION): 補間方法 (関節空間/位置空間)
            interference(PyBulletInteference): 干渉判定クラス
            joins_limits(numpy.ndarray): 関節限界 (0列目：最小値, 1列目：最大値)
            weight(numpy.ndarray): 各次元の重み

        戻り値
            bool: True/False = 経路生成の成功/失敗
        """
        # 戻り値
        result = False

        # 経路生成の準備
        self._preparation_planning(start_pos, end_pos, interpolation, interference, joints_limit)

        # 始点ノードをツリーに追加
        self._add_node_start_tree(start_pos, INITIAL_NODE_NEAR_NODE)

        # 経路生成は一定時間内に終了させる
        start_time = time.time()

        counter = 0

        # 終点までの経路生成が終わるまでループ
        while True:
            counter += 1
            # ランダムな位置を取得
            random_pos = self._get_random_pos(end_pos)
            # ランダムな位置と最短ノードを計算
            near_node  = self._start_tree.get_near_node(random_pos, weight)
            # 最短ノードの位置を取得
            near_node_pos = self._start_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
            # 最短ノードからランダムな値方向へ新しいノード(位置)を作成
            new_node_pos  = self._calc_new_pos(random_pos, near_node_pos, weight)

            # 新規ノードと最短ノード間での干渉判定
            is_interference = self._interference.is_line_interference(new_node_pos, near_node_pos)
            if not is_interference:
                # 干渉なしのため，ノード追加
                self._add_node_start_tree(new_node_pos, near_node)

                # 終点との距離が一定以内であるかの確認
                if self._chk_end_pos_dist(new_node_pos, end_pos):
                    # 一定範囲内のため，経路生成の完了
                    result = True
                    break

            now_time = time.time()
            if now_time - start_time >= self._TIMEOUT_VALUE:
                # タイムアウト
                break

        if result:
            # 経路生成に成功のため，経路生成の後処理
            self._fin_planning(start_pos, end_pos)

        return result

    def _add_node_start_tree(self, pos, near_node):
        """
        始点ツリーにノードを追加

        パラメータ
            pos(numpy.ndarray): 位置
            near_node(int): 最短ノード
        """
        # _start_treeにノードを追加
        node = np.append(pos, near_node).reshape(1, -1)
        self._start_tree.add_node(node)

    def _chk_end_pos_dist(self, pos, target_pos):
        """
        目標点との距離が一定範囲内かつ干渉なしであるかの確認

        パラメータ
            pos(numpy.ndarray): ノード位置
            end_pos(numpy.ndarray): 目標点

        戻り値
            bool: True / False = 一定範囲内かつ干渉なしである / でない
        """
        # 戻り値
        is_near = False

        # 距離を計算
        dist = np.linalg.norm(target_pos - pos)

        # 一定範囲内かつ干渉なしかの確認
        if dist <= self._moving_value:  # 一定距離内
            if not self._interference.is_line_interference(pos, target_pos):    # 干渉なし
                is_near = True

        return is_near

    def _fin_planning(self, start_pos, end_pos):
        """
        経路生成の終了処理

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
        """
        # 始点から終点までの経路に関係するノードを選択
        revers_path = end_pos.reshape(1, -1)
        near_node   = -1

        while True:
            # 終点から始点方向へノードを取得
            node = self._start_tree.nodes[near_node]
            pos  = node[:self._NODE_NEAR_NODE_IDX].reshape(1, -1)
            # 浮動小数型になっているので，整数型に型変換
            near_node   = int(node[self._NODE_NEAR_NODE_IDX])
            revers_path = np.append(revers_path, pos, axis=0)
            if near_node == INITIAL_NODE_NEAR_NODE:
                # 始点ノードまで取得できたため，処理終了
                break

        # 経路が終点からの順番になっているため，始点から終点とする
        self._pathes = revers_path[::-1]
        # ツリーに終点を追加 (要素番号を指定するため -1)
        self._add_node_start_tree(end_pos, self._start_tree.nodes.shape[0] - 1)

    def _calc_new_pos(self, random_pos, near_node_pos, weight):
        """
        最短ノードからランダムな値方向へ新しいノード(位置)を作成

        パラメータ
            random_pos(numpy.ndarray): ランダムな位置
            near_node_pos(numpy.ndarray): 最短ノード位置
            weight(numpy.ndarray): 各次元の重み

        戻り値
            new_pos(numpy.ndarray): 新しいノード
        """
        # 方向を計算
        direction = random_pos - near_node_pos

        # 修正後 ↓
        # 重みの計算
        if weight is None:
            weight = 1

        weighted_direction = direction * weight
        norm_direction = weighted_direction / (np.linalg.norm(weighted_direction) + EPSILON)
        # 修正後 ↑

        # 新しいノードを作成
        new_pos = near_node_pos + norm_direction * self._moving_value

        return new_pos

    def _get_random_pos(self, target_pos):
        """
        ランダムな位置を取得

        パラメータ
            target_pos(numpy.ndarray): 目標点

        戻り値
            random_pos(numpy.ndarray): ランダムな位置
        """
        # 乱数を取って，目標点を選択するかランダムを選択するか
        select_goal = np.random.rand()

        if select_goal < self._GOAL_SAMPLE_RATE:
            # 目標点を選択s
            random_pos = target_pos
        else:
            random_pos = np.random.uniform(self._strict_min_pos, self._strict_max_pos)

        return random_pos


    def set_strict_pos(self, min_pos, max_pos):
        """
        探索範囲の設定

        パラメータ
            min_pos(numpy.ndarray): 探索の最小範囲
            max_pos(numpy.ndarray): 探索の最大範囲
        """
        # パラメータの要素数確認
        if min_pos.shape[0] != max_pos.shape[0]:
            raise ValueError(f"min_pos'shape and max_pos'shape are not match. min_pos'shape is {min_pos.shape}. max_pos'shape is {max_pos.shape}")
        if min_pos.shape[0] != self._strict_min_pos.shape[0]:
            raise ValueError(f"min_pos'shape and _strict_min_pos'shape are not match. min_pos'shape is {min_pos.shape[0]}. _strict_min_pos'shape is {self._strict_min_pos.shape[0]}")

        # プロパティの更新
        self._strict_min_pos = min_pos
        self._strict_max_pos = max_pos

    def _reset(self):
        """
        データの初期化
        """
        self._start_tree.reset()

        if len(self._pathes) != 0:
            # 何かしらのデータが保存
            del self._pathes
        self._pathes = []

        self._interpolation = INTERPOLATION.NONE.value
        self._dim = DIMENTION_NONE

    def _set_interpolation(self, interpolation):
        """
        経路生成したい探索空間の設定

        パラメータ
            interpolation(int): 補間の種類 (関節補間/位置補間)
        """
        if interpolation == INTERPOLATION.POSITION.value:
            # 位置空間
            self._moving_value = self._MOVING_VALUE_POS
        elif interpolation == INTERPOLATION.JOINT.value:
            # 関節空間
            self._moving_value = self._MOVING_VALUE_JOINT
        else:
            # 異常値
            raise ValueError(f"interpolation is abnormal. interpolation is {interpolation}")

        # 補間種類の更新
        self._interpolation = interpolation

    def _strict_planning_pos(self, start_pos, end_pos, joints_limit):
        """
        探索範囲を制限する

        パラメータ
            start_pos(numpy.ndarray): 始点
            end_pos(numpy.ndarray): 終点
            joints_limit(numpy.ndarray): 関節限界 (0列目：最小値, 1列目：最大値)
        """
        all_pos = np.array([start_pos, end_pos])
        # 各列の最大/最小値を取得
        min_pos = np.min(all_pos, axis=0)
        max_pos = np.max(all_pos, axis=0)

        if self._interpolation == INTERPOLATION.POSITION:
            # 位置空間の探索
            self._strict_min_pos = min_pos - self._STRICT_PLANNING_ROB_POS
            self._strict_max_pos = max_pos + self._STRICT_PLANNING_ROB_POS
        else:
            # 関節空間の探索
            # 探索の最小値を関節の最小値以上に制限
            min_joints = np.array([min_pos - self._STRICT_PLANNING_ROB_JOINT, joints_limit[:, 0]])
            self._strict_min_pos = np.max(min_joints, axis=0)
            # 探索の最大値を関節の最大値以下に制限
            max_joints = np.array([max_pos + self._STRICT_PLANNING_ROB_JOINT, joints_limit[:, 1]])
            self._strict_max_pos = np.min(max_joints, axis=0)


    def _make_folder(self, folder_name):
        """
        フォルダーの作成

        パラメータ
            folder_name(str): 作成したいフォルダー名
        """
        # フォルダーが作成済みでもエラーを出力しないよう，exist_ok=Trueとした．
        os.makedirs(folder_name, exist_ok=True)

    def _reset_folder(self, interpolation):
        """
        フォルダー内のファイルを全削除

        パラメータ
            interpolation(str): 探索方法 (位置空間/関節空間)
        """
        # フォルダー名を取得
        folder_names = self._get_path_plan_folders(interpolation)
        for folder_name in folder_names:
            for entry in os.listdir(folder_name):
                full_path = os.path.join(folder_name, entry)
                if os.path.isfile(full_path) or os.path.islink(full_path):
                    os.remove(full_path)         # 通常ファイル・シンボリックリンクを削除

    def _get_path_plan_folders(self, interpolation):
        """
        経路生成結果を保存する複数のフォルダー名を取得

        パラメータ
            interpolation(str): 探索方法 (位置空間/関節空間)

        戻り値
            folder_names(list): 複数のフォルダー名
        """
        folder_names = [os.path.join(self._name, interpolation), ]

        return folder_names

    def _make_path_plan_folder(self, interpolation):
        """
        経路生成結果を保存するフォルダー作成

        パラメータ
            interpolation(str): 探索方法 (位置空間/関節空間)
        """
        # フォルダー名を取得
        folder_names = self._get_path_plan_folders(interpolation)
        for folder_name in folder_names:
            self._make_folder(folder_name)

    def save(self):
        """
        生成した経路をファイル保存
        """
        # 始点から終点までの修正済みの経路をファイル保存
        self._save_numpy_data_to_txt(self._pathes, self._FILE_NAME_PATHES)
        # 始点のツリーを保存
        self._save_numpy_data_to_txt(self._start_tree.nodes, self._FILE_NAME_START_TREE)

    def _save_numpy_data_to_txt(self, data, file_name):
        """
        Numpyデータをテキストファイルに保存

        パラメータ
            data(numpy.ndarray): ファイル保存したいデータ
            file_name(str): 保存したいファイル名
        """
        # 引数の確認
        if len(data) == 0:
            # データが存在しないため，処理終了
            return

        if not file_name:
            # ファイル名が存在しないため，処理終了
            return

        # ファイル名にフォルダ名を追加 (各経路生成手法で異なるフォルダにデータを保存)
        full_path = f"{self._name}/{self._interpolation}/{file_name}"

        np.savetxt(full_path, data)


class _PyBulletRRTConnect(_PyBulletRRT):
    # 定数の定義
    # 経路生成の名前
    _PATH_PLAN = PATHPLAN.RRTCONNECT.value

    # ツリーの要素番号を定義
    _NODE_NEAR_NODE_IDX = RRTCONNECT_NEAR_NODE_IDX


    def __init__(self):
        """
        コンストラクタ
        """
        # 親クラスのコンストラクタを呼ぶ
        super().__init__()
        # プロパティの初期化
        self._end_tree = _Tree(self._NODE_NEAR_NODE_IDX)
        self._state  = RRTCONNECTSTATE.STREE_RAND.value
        self._states = []


    def planning(self, start_pos, end_pos, interpolation, interference, joints_limit, weight=None):
        """
        始点から終点までの経路生成の実行

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            interpolation(INTERPOLATION): 補間方法 (関節空間/位置空間)
            interference(PyBulletInteference): 干渉判定クラス
            joins_limits(numpy.ndarray): 関節限界 (0列目：最小値, 1列目：最大値)
            weight(numpy.ndarray): 各次元の重み

        戻り値
            bool: True/False = 経路生成の成功/失敗
        """
        # 戻り値
        result = False

        # 経路生成の準備
        self._preparation_planning(start_pos, end_pos, interpolation, interference, joints_limit)

        # 始点ノードをツリーに追加
        self._add_node_start_tree(start_pos, INITIAL_NODE_NEAR_NODE)
        # 終点ノードをツリーに追加
        self._add_node_end_tree(end_pos, INITIAL_NODE_NEAR_NODE)

        # 経路生成は一定時間内に終了させる
        start_time = time.time()

        counter = 0

        # 始点から終点までの経路が生成できるまでループ
        while True:
            counter += 1
            # 1処理実施して，経路生成が完了したか確認する
            if self._exec_one_step(start_pos, end_pos, weight):
                # 経路生成の完了
                result = True
                break

            # 干渉の有無に関わらずにタイムアウトの確認
            now_time = time.time()
            if now_time - start_time >= self._TIMEOUT_VALUE:
                # タイムアウト
                break

        if result:
            # 経路生成に成功のため，経路生成の後処理
            self._fin_planning(start_pos, end_pos)

        return result

    def _state_transition(self):
        """
        状態を遷移させる
        """
        if self._state == RRTCONNECTSTATE.STREE_RAND.value:
            # 始点ツリーにランダム点を追加する状態
            self._state = RRTCONNECTSTATE.ETREE_TO_STREE.value

        elif self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value:
            # 始点ツリーに終点ツリーで作成したノードへ伸ばす状態
            self._state = RRTCONNECTSTATE.STREE_RAND.value

        elif self._state == RRTCONNECTSTATE.ETREE_RAND.value:
            # 終点ツリーにランダム点を追加する状態
            self._state = RRTCONNECTSTATE.STREE_TO_ETREE.value

        else:
            # 終点ツリーに始点ツリーで作成したノードへ伸ばす状態
            self._state = RRTCONNECTSTATE.ETREE_RAND.value

    def _exec_one_step(self, start_pos, end_pos, weight):
        """
        1ステップの処理を実施 (探索を1回実装)

        パラメータ
            start_pos(numpy.ndarray): 始点
            end_pos(numpy.ndarray): 終点
            weight(numpy.ndarray): 各次元の重み

        戻り値
            bool: True/False = 経路生成の完了 / 未完了
        """
        # 戻り値
        complete_path = False

        # 最短ノードを算出したいツリーを取得
        if self._state == RRTCONNECTSTATE.STREE_RAND.value or self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value:
            # 始点ツリーにランダム点を追加する状態または始点ツリーに終点ツリーで作成したノードへ伸ばす状態
            my_tree = self._start_tree
            your_tree = self._end_tree
            target_pos = end_pos
            set_tree_func = self._add_node_start_tree
        else:
            my_tree = self._end_tree
            your_tree = self._start_tree
            target_pos = start_pos
            set_tree_func = self._add_node_end_tree

        # ツリーにランダム点を追加
        if self._state == RRTCONNECTSTATE.STREE_RAND.value or self._state == RRTCONNECTSTATE.ETREE_RAND.value:
            # 始点ツリーにランダム点を追加する状態または終点ツリーにランダム点を追加する状態
            # ランダムな値を取得する
            random_pos = self._get_random_pos(target_pos)
            # ランダムな値とツリーの最短ノードを計算
            near_node = my_tree.get_near_node(random_pos, weight)
            # 位置と最短ノードが保存されているから，位置だけを取得
            near_node_pos = my_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
            # 最短ノードからランダムな値方向へ新しいノードを作成
            new_node_pos = self._calc_new_pos(random_pos, near_node_pos, weight)

            # 干渉物との干渉判定
            is_interference = self._interference.is_line_interference(new_node_pos, near_node_pos)
            if not is_interference:  # 干渉なし
                # ツリーにノードを追加
                set_tree_func(new_node_pos, near_node)
                # 状態を遷移させる
                self._state_transition()

        else:
            # ツリーで作成したノードを取得
            target_pos = your_tree.nodes[-1, :self._NODE_NEAR_NODE_IDX]
            # ツリー内の最短ノードを計算
            near_node = my_tree.get_near_node(target_pos, weight)
            # 位置と最短ノードが保存されているから，位置だけを取得
            near_node_pos = my_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
            # 最短ノードから終点ツリーのノード方向へ新しいノードを作成
            new_node_pos = self._calc_new_pos(target_pos, near_node_pos, weight)

            # 干渉物との干渉判定
            is_interference = self._interference.is_line_interference(new_node_pos, new_node_pos)
            if is_interference:  # 干渉あり
                # 状態を遷移させる
                self._state_transition()
            else:   # 干渉なし
                # ツリーにノードを追加
                set_tree_func(new_node_pos, near_node)
                # 始点から終点までの経路が生成できたかの確認
                if self._chk_end_pos_dist(new_node_pos, target_pos):
                    # 経路生成の完了
                    complete_path = True

        return complete_path

    def _add_node_end_tree(self, pos, near_node):
        """
        終点ツリーにノードを追加

        パラメータ
            pos(numpy.ndarray): 位置
            near_node(int): 最短ノード
        """
        # _end_treeにノードを追加
        node = np.append(pos, near_node).reshape(1, -1)
        self._end_tree.add_node(node)

    def _fin_planning(self, start_pos, end_pos):
        """
        経路生成の終了処理

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
        """
        # 始点ツリーから，終点ツリーと繋がったパスを取得
        start_path = []
        near_node  = -1

        while True:
            # 終点ツリーと繋がったノードから始点へノードを取得
            node = self._start_tree.nodes[near_node]
            pos  = node[:self._NODE_NEAR_NODE_IDX].reshape(1, -1)
            # 浮動小数点を整数型に型変換
            near_node = int(node[self._NODE_NEAR_NODE_IDX])

            if len(start_path) == 0:
                # 初回だけ
                start_path = pos
            else:
                start_path = np.append(start_path, pos, axis=0)

            if near_node == INITIAL_NODE_NEAR_NODE:
                # 始点ノードまで取得できたため，ループ終了
                break

        # 始点ツリーのパスは，逆順(終点ツリーと繋がったノードから始点ノードの順番)に保存されているから，正順に変更
        self._pathes = start_path[::-1]

        # 終点ツリーから，始点ツリーと繋がったパスを取得
        end_path  = []
        near_node = -1

        while True:
            # 始点ツリーと繋がったノードから終点へノードを取得
            node = self._end_tree.nodes[near_node]
            pos  = node[:self._NODE_NEAR_NODE_IDX].reshape(1, -1)
            # 浮動小数型になっているので，整数型に型変換
            near_node = int(node[self._NODE_NEAR_NODE_IDX])

            if len(end_path) == 0:
                # 初回だけ
                end_path = pos
            else:
                end_path = np.append(end_path, pos, axis=0)

            if near_node == INITIAL_NODE_NEAR_NODE:
                # 終点ノードまで取得できたため，ループ終了
                break

        # 始点から終点までの経路を保存
        self._pathes = np.append(self._pathes, end_path, axis=0)


    def _reset(self):
        """
        データの初期化
        """
        # 親クラスの処理を実装
        super()._reset()

        # 終点ツリーの初期化
        self._end_tree.reset()

        # 状態の初期化
        self._state = RRTCONNECTSTATE.STREE_RAND.value
        if len(self._states) != 0:
            # 何かしらのデータ保存
            del self._states
        self._states = []



class _PyBulletRRTStar(_PyBulletRRT):
    # 定数の定義
    # 経路生成の名前
    _PATH_PLAN = PATHPLAN.RRTSTAR.value

    # ツリーの要素番号を定義
    _NODE_NEAR_NODE_IDX = RRTSTAR_NEAR_NODE_IDX     # 最短ノードの列番号
    _NODE_COST_IDX      = RRTSTAR_COST_IDX          # コストの列番号

    # その他
    _MAX_ITERATE = 3000             # 最大探索回数
    _TIMEOUT_VALUE = 600            # タイムアウト
    _NEAR_NODE_RADIUS_COEF = 10     # 近傍ノードとする探索球の半径の係数


    def __init__(self):
        """
        コンストラクタ
        """
        # プロパティの初期化
        self._name = self._PATH_PLAN
        self._start_tree = _CostedTree(self._NODE_NEAR_NODE_IDX, self._NODE_COST_IDX)
        self._pathes = []
        self._max_iterate = self._MAX_ITERATE
        self._near_node_radius = 0


    def planning(self, start_pos, end_pos, interpolation, interference, joints_limit, weight=None):
        """
        始点から終点までの経路生成の実行

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            interpolation(INTERPOLATION): 補間方法 (関節空間/位置空間)
            interference(PyBulletInteference): 干渉判定クラス
            joins_limits(numpy.ndarray): 関節限界 (0列目：最小値, 1列目：最大値)
            weight(numpy.ndarray): 各次元の重み

        戻り値
            bool: True/False = 経路生成の成功/失敗
        """
        # 戻り値
        result = False

        # 経路生成の準備
        self._preparation_planning(start_pos, end_pos, interpolation, interference, joints_limit)

        # 始点ノードをツリーに追加
        self._add_node_start_tree(start_pos, INITIAL_NODE_NEAR_NODE, 0.0)

        # 近傍ノードを定義する半径
        self._near_node_radius = self._moving_value * self._NEAR_NODE_RADIUS_COEF

        # 経路生成は一定時間内に終了させる
        start_time = time.time()

        # 全探索回数ループ
        for counter in range(self._max_iterate):
            # ランダムな値を取得
            random_pos = self._get_random_pos(end_pos)
            # ランダムな値と最短ノードを計算
            near_node  = self._start_tree.get_near_node(random_pos, weight)
            # 最短ノード位置を取得
            near_node_pos = self._start_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
            # 最短ノードからランダムな値方向へ新しいノード(位置)を作成
            new_node_pos  = self._calc_new_pos(random_pos, near_node_pos, weight)

            # 新規ノードと最短ノード間での干渉判定
            is_interference = self._interference.is_line_interference(new_node_pos, near_node_pos)
            if not is_interference:
                # 干渉なしのため，データを設定する

                # 新規箇所 ↓
                # 新規ノードのコストを計算
                new_cost = self._start_tree.calc_cost(near_node, new_node_pos)
                # 近傍ノードを全部取得
                near_node_list = self._get_near_node_list(new_node_pos)
                # 近傍ノードからコストが最小となるノードを取得
                min_cost_node, min_cost = self._get_min_cost_node(new_node_pos, near_node_list, near_node, new_cost)
                # 近傍ノード内でコストが小さくなれば，最短ノードを更新
                self._update_near_node(new_node_pos, near_node_list, min_cost)
                # 新規箇所 ↑

                # ノードを設定
                self._add_node_start_tree(new_node_pos, min_cost_node, min_cost)
                # 終点との距離が一定範囲内であるかの確認
                if self._chk_end_pos_dist(new_node_pos, end_pos):
                    # 一定範囲内のため，戻り値を成功に更新
                    result = True

            # 干渉の有無に関わらずにタイムアウトの確認はする
            now_time = time.time()
            if now_time - start_time >= self._TIMEOUT_VALUE:
                # タイムアウト
                print(f"timeout counter = {counter}")
                break

        if result:
            # 経路生成に成功のため，経路生成の終了処理
            self._fin_planning(start_pos, end_pos, weight)

        return result

    def _add_node_start_tree(self, pos, near_node, cost):
        """
        始点ツリーにノードを追加

        パラメータ
            pos(numpy.ndarray): 位置
            near_node(numpy.ndarray): ランダムな位置
            cost(int): 最短ノード
        """
        # ノード作成
        array_data = np.array([near_node, cost])
        node = np.append(pos, array_data).reshape(1, -1)

        # ツリーにノードを追加
        self._start_tree.add_node(node)

    def _calc_cost(self, pos, near_node):
        """
        ノードのコストを計算

        パラメータ
            pos(numpy.ndarray): 新規ノード位置
            near_node(int): 新規ノードとの最短ノード

        戻り値
            cost(float): コスト
        """
        # 最短ノードの情報を取得
        node = self._start_tree.nodes[near_node]
        node_pos  = node[:self._NODE_NEAR_NODE_IDX]
        node_cost = node[self._NODE_COST_IDX]

        # 最短ノードと新規ノードのコスト(距離)を算出
        cost  = np.linalg.norm(node_pos - pos)
        cost += node_cost

        return cost

    def _get_near_node_list(self, pos):
        """
        ノードと近傍ノードをリストで取得

        パラメータ
            pos(numpy.ndarray): ノード位置

        戻り値
            near_node_list(list): 近傍ノードリスト
        """
        # 近傍の閾値はノード数に依存
        n_node = self._start_tree.nodes.shape[0]
        threshold = self._near_node_radius * np.power(np.log(n_node) / n_node, 1 / self._dim)

        # 閾値以内の距離であるノードを全部取得
        near_node_list = self._start_tree.get_near_node_list(pos, threshold)

        return near_node_list

    def _get_min_cost_node(self, pos, near_node_list, near_node, near_node_cost):
        """
        コストが最小となる近傍ノードを取得

        パラメータ
            pos(numpy.ndarray): ノード位置
            near_node_list(list): 近傍ノードリスト
            near_node(int): 最短ノード
            near_node_cost(float): 最短ノード追加時のコスト

        戻り値
            min_cost_node(int): コスト最小ノード
            min_cost(float): コスト最小値
        """
        # 戻り値の初期化
        min_cost = near_node_cost
        min_cost_node = near_node

        if not near_node_list:
            # 近傍ノードがないため，処理終了
            return min_cost_node, min_cost

        for node_idx in near_node_list:
            # 近傍ノードに関する情報を保存
            node = self._start_tree.nodes[node_idx]
            node_pos  = node[:self._NODE_NEAR_NODE_IDX]
            node_cost = node[self._NODE_COST_IDX]

            # コスト(距離)を計算
            distance = np.linalg.norm(node_pos - pos)

            # 2点間の干渉チェック
            is_interference = self._interference.is_line_interference(node_pos, pos)
            if not is_interference:
                # 干渉なし
                new_cost = distance + node_cost
                if new_cost < min_cost:
                    # 小さくなるため，最短ノードを修正
                    min_cost_node = node_idx
                    min_cost = new_cost

        return min_cost_node, min_cost

    def _update_near_node(self, pos, near_node_list, cost):
        """
        近傍ノード内の親ノードを更新

        パラメータ
            pos(numpy.ndarray): ノード位置
            near_node_list(list): 近傍ノードリスト
            cost(float): コスト
        """
        if not near_node_list:
            # 近傍ノードがないため，処理終了
            return

        # 新規ノードのインデックス番号を保存
        new_node_idx = self._start_tree.nodes.shape[0]

        for node_idx in near_node_list:
            # 近傍ノードに関する情報を取得
            node = self._start_tree.nodes[node_idx]
            node_pos  = node[:self._NODE_NEAR_NODE_IDX]
            node_cost = node[self._NODE_COST_IDX]

            # コスト(距離)を計算
            distance = np.linalg.norm(node_pos - pos)

            # 2点間の干渉チェック
            is_interference = self._interference.is_line_interference(node_pos, pos)
            if not is_interference:
                # 干渉なし
                new_cost = distance + cost
                if new_cost < node_cost:
                    # 小さくなるため，最短ノードを修正
                    self._start_tree.chg_node_info(node_idx, new_node_idx, new_cost)

    def _fin_planning(self, start_pos, end_pos, weight):
        """
        経路生成の終了処理

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
            weight(numpy.ndarray): 各次元の重み
        """
        # 始点から終点までの経路に関係するノードを選択
        revers_path = end_pos.reshape(1, -1)
        # 終点との最短ノードを取得
        near_node = self._start_tree.get_near_node(end_pos, weight)
        # 最短ノードとのコストを計算
        near_node_cost = self._calc_cost(end_pos, near_node)
        # 近傍ノードを全部取得
        near_node_list = self._get_near_node_list(end_pos)
        # 近傍ノードからコストが最小となるノードを取得
        min_cost_node, _ = self._get_min_cost_node(end_pos, near_node_list, near_node, near_node_cost)
        near_node = min_cost_node

        while True:
            # 終点から始点方向へノードを取得
            node = self._start_tree.nodes[near_node]
            pos  = node[:self._NODE_NEAR_NODE_IDX].reshape(1, -1)
            # 浮動小数点になっているので，整数型に型変換
            near_node   = int(node[self._NODE_NEAR_NODE_IDX])
            revers_path = np.append(revers_path, pos, axis=0)
            if near_node == INITIAL_NODE_NEAR_NODE:
                # 始点ノードまで取得できたため，処理終了
                break

        # 経路が始点からの順番になっているため，始点から終点とする
        self._pathes = revers_path[::-1]
        # ツリーに終点を追加 (終点位置 + 終点の親ノード + コスト)
        self._add_node_start_tree(end_pos, min_cost_node, 0.0)

    def _get_unit_ball(self):
        """
        単位球内の位置を取得

        戻り値
            position(numpy.ndarray): 単位球内の位置
        """
        lenth_of_one_side = 1.0
        while True:
            # -1から1の範囲で，適当な座標を取得
            position = np.random.uniform(np.ones(self._dim) * lenth_of_one_side * -1, np.ones(self._dim) * lenth_of_one_side)
            # 距離を計算
            distance = np.linalg.norm(position)
            if distance <= 1:
                # 単位球の中をプロット
                # 単位球の中をプロットする確率は 単位球の体積 / 一辺が2の立方体の体積
                break
            else:
                # 単位球の外をプロット
                # プロットできなかった時は，辺の長さを小さくして，単位球の中をプロットしやすくする (whileループから早く抜けたいから)
                lenth_of_one_side -= 0.0001

        return position


class PyBulletRRTController:
    def __init__(self, path_plan):
        """
        コンストラクタ

        パラメータ
            path_plan(PATHPLAN): 経路生成手法
            interference(PyBulletInteference): 干渉判定クラス
        """
        # プロパティの初期化
        self.__rrt = self.__get_each_path_plan_instance(path_plan)


    @property
    def pathes(self):
        """
        始点から終点までの経路
        
        戻り値
            numpy.ndarray: 始点から終点までの経路
        """
        return self.__rrt.pathes


    def set_strict_pos(self, min_pos, max_pos):
        """
        探索範囲の設定

        パラメータ
            min_pos(numpy.ndarray): 探索の最小範囲
            max_pos(numpy.ndarray): 探索の最大範囲
        """
        self.__rrt.set_strict_pos(min_pos, max_pos)


    def save(self):
        """
        生成した経路をファイル保存
        """
        self.__rrt.save()


    def planning(self, start_pos, end_pos, interpolation, interference, joints_limit, weight=None):
        """
        始点から終点までの経路生成の実行

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            interpolation(INTERPOLATION): 補間方法 (関節空間/位置空間)
            interference(PyBulletInteference): 干渉判定クラス
            joins_limits(numpy.ndarray): 関節限界 (0列目：最小値, 1列目：最大値)
            weight(numpy.ndarray): 各次元の重み

        戻り値
            bool: True/False = 経路生成の成功/失敗
        """
        return self.__rrt.planning(start_pos, end_pos, interpolation, interference, joints_limit, weight)


    def reset(self, path_plan):
        """
        データの初期化
        
        パラメータ
            path_plan(PATHPLAN): 経路生成手法
        """
        if self.__rrt is not None:
            # インスタンス作成済みの場合
            del self.__rrt
            self.__rrt = None

        # 再度インスタンスの作成
        self.__rrt = self.__get_each_path_plan_instance(path_plan)

    def __get_each_path_plan_instance(self, path_plan):
        """
        経路生成手法に応じたクラスのインスタンスを取得

        パラメータ
            path_plan(str): 経路生成手法名

        戻り値
            rrt: 各経路生成手法のクラス
        """
        # 経路生成手法によって，アニメーションの分岐を分ける
        if path_plan == PATHPLAN.RRT.value:
            # RRT
            rrt_cls = _PyBulletRRT
        elif path_plan == PATHPLAN.RRTCONNECT.value:
            # RRT-Connect
            rrt_cls = _PyBulletRRTConnect
        elif path_plan == PATHPLAN.RRTSTAR.value:
            # RRT*
            rrt_cls = _PyBulletRRTStar
        else:
            # 異常
            raise ValueError(f"path_plan is abnormal. path_plan is {path_plan}")

        return rrt_cls()
