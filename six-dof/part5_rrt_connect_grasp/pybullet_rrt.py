# 経路生成手法であるRRT (Rapidly-exploring Random Tree) の実装 (PyBullet用)


# ライブラリの読み込み
import numpy as np
import os


# 自作モジュールの読み込み
from constant import *              # 定数



class Tree:
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


class RRTPyBullet:
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

    _GOAL_SAMPLE_RATE = 0.1     # ランダムな値を取るときに，終点を選択する確率


    def __init__(self):
        """
        コンストラクタ
        """
        self._name   = self._PATH_PLAN
        self._pathes = []
        self._start_tree    = Tree(self._NODE_NEAR_NODE_IDX)
        self._interpolation = INTERPOLATION.NONE.value
        self._moving_value  = self._MOVING_VALUE_JOINT
        self._dim = DIMENTION_NONE


    @property
    def name(self):
        """
        _nameプロパティのゲッター
        """
        return self._name

    @property
    def pathes(self):
        """
        _pathesプロパティのゲッター
        """
        return self._pathes


    def expand_once(self, start_pos, end_pos, weight=None):
        """
        新規ノードおよび最短ノードの取得 (干渉判定は実装しない)

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
            weight(numpy.ndarray): 各次元の重み

        戻り値
            new_node_pos(numpy.ndarray): 新規ノード
            near_node_pos(numpy.ndarray): 最短ノード位置
            near_node(int): 最短ノード番号
        """
        # ランダムな値を取得
        random_pos = self._get_random_pos(end_pos)
        # ランダムな値と最短ノードを計算
        near_node  = self._start_tree.get_near_node(random_pos, weight)
        # 最短ノードの位置
        near_node_pos = self._start_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
        # 最短ノードからランダムな値方向へ新しいノード(位置)を作成
        new_node_pos  = self._calc_new_pos(random_pos, near_node_pos, weight)

        return new_node_pos, near_node_pos, near_node

    def add_node_and_chk_goal(self, start_pos, end_pos, node_pos, near_node):
        """
        ノード追加 + 経路生成の完了確認

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            node_pos(numpy.ndarray): ノード位置
            near_node(int): 親ノード

        戻り値
            is_successful(bool): True/False = 経路生成の完了/未完了
        """
        # 処理結果
        is_successful = False

        # 始点ツリーにノードを追加
        self._add_node_start_tree(node_pos, near_node)

        # 終点との距離が一定範囲内であるかの確認
        if self._chk_end_pos_dist(node_pos, end_pos):
            # 一定範囲内のため，経路生成の完了
            is_successful = True

        return is_successful

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

    def _chk_end_pos_dist(self, pos, end_pos):
        """
        終点との距離が一定範囲内であるかの確認

        パラメータ
            pos(numpy.ndarray): ノード位置
            end_pos(numpy.ndarray): 経路生成の終点

        戻り値
            is_near(bool): True / False = 一定範囲内である / でない
        """
        is_near = False
        # 距離を計算
        dist = np.linalg.norm(end_pos - pos)
        # 一定範囲内であるかの確認
        if dist <= self._moving_value:
            is_near = True

        return is_near

    def fin_planning(self, start_pos, end_pos):
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

        # 重みの計算
        if weight is None:
            weight = 1
        weighted_direction = direction * weight
        norm_direction = weighted_direction / (np.linalg.norm(weighted_direction) + EPSILON)

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

    def handle_interference(self):
        """
        干渉判定時の処理
        """
        # 何もしない
        pass


    def preparation(self, start_pos, end_pos, interpolation):
        """
        経路生成の準備

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
            interpolation(int): 補間方法 (関節空間/位置空間)
        """
        # データの初期化
        self._reset()

        # 始点と終点の次元数が一致しているかの確認
        start_pos_dim = np.size(start_pos)
        end_pos_dim   = np.size(end_pos)
        if start_pos_dim != end_pos_dim:
            # 次元数が異なるので異常
            raise ValueError(f"start_pos_dim and end_pos_dim are not matched. start_pos_dim is {start_pos_dim}, end_pos_dim is {end_pos_dim}")

        self._dim = start_pos_dim

        # 探索空間の設定
        self._set_interpolation(interpolation)

        # 始点ノードをツリーに追加
        self._add_node_start_tree(start_pos, INITIAL_NODE_NEAR_NODE)

        # 探索範囲を設定
        self._strict_planning_pos(start_pos, end_pos)

        # 結果を保存するフォルダ作成
        self._make_path_plan_folder(interpolation)

        # フォルダー内のファイルを全部削除
        self._reset_folder(interpolation)

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

    def _strict_planning_pos(self, start_pos, end_pos):
        """
        探索範囲を制限する

        パラメータ
            start_pos(numpy.ndarray): 始点
            end_pos(numpy.ndarray): 終点
        """
        all_pos = np.array([start_pos, end_pos])
        # 各列の最大/最小値を取得
        min_pos = np.min(all_pos, axis=0)
        max_pos = np.max(all_pos, axis=0)

        if self._interpolation == INTERPOLATION.POSITION:
            # 位置空間の探索
            strict_planning_pos = self._STRICT_PLANNING_ROB_POS
        else:
            # 関節空間の探索
            strict_planning_pos = self._STRICT_PLANNING_ROB_JOINT

        self._strict_min_pos = min_pos - strict_planning_pos
        self._strict_max_pos = max_pos + strict_planning_pos


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


class RRTConnectPyBullet(RRTPyBullet):
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
        self._end_tree = Tree(self._NODE_NEAR_NODE_IDX)
        self._state  = RRTCONNECTSTATE.STREE_RAND.value
        self._states = []


    @property
    def start_tree(self):
        """
        _start_treeのゲッター
        """
        return self._start_tree

    @property
    def end_tree(self):
        """
        _end_treeのゲッター
        """
        return self._end_tree


    def expand_once(self, start_pos, end_pos, weight=None):
        """
        新規ノードおよび最短ノードの取得 (干渉判定は実装しない)

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
            weight(numpy.ndarray): 各次元の重み

        戻り値
            new_node_pos(numpy.ndarray): 新規ノード
            near_node_pos(numpy.ndarray): 最短ノード位置
            near_node(int): 最短ノード番号
        """
        # 1回分の処理を実装
        return self._exec_state(start_pos, end_pos, weight)

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

    def _exec_state(self, start_pos, end_pos, weight):
        """
        状態(_state)に応じた処理を実施

        パラメータ
            start_pos(numpy.ndarray): 始点
            end_pos(numpy.ndarray): 終点
            weight(numpy.ndarray): 各次元の重み

        戻り値
            new_node_pos(numpy.ndarray): 新規ノード
            near_node_pos(numpy.ndarray): 最短ノード位置
            near_node(int): 最短ノード番号
        """
        # 最短ノードを算出したいツリーを取得
        if self._state == RRTCONNECTSTATE.STREE_RAND.value or self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value:
            # 始点ツリーにランダム点を追加する状態または始点ツリーに終点ツリーで作成したノードへ伸ばす状態
            my_tree = self._start_tree
            your_tree = self._end_tree
            target_pos = end_pos
        else:
            my_tree = self._end_tree
            your_tree = self._start_tree
            target_pos = start_pos

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

        else:
            # ツリーで作成したノードを取得
            target_pos = your_tree.nodes[-1, :self._NODE_NEAR_NODE_IDX]
            # ツリー内の最短ノードを計算
            near_node = my_tree.get_near_node(target_pos, weight)
            # 位置と最短ノードが保存されているから，位置だけを取得
            near_node_pos = my_tree.nodes[near_node, :self._NODE_NEAR_NODE_IDX]
            # 最短ノードから終点ツリーのノード方向へ新しいノードを作成
            new_node_pos = self._calc_new_pos(target_pos, near_node_pos, weight)

        return new_node_pos, near_node_pos, near_node

    def handle_interference(self):
        """
        干渉判定時の処理
        """
        # 始点ツリーまたは終点ツリーを伸ばす状態
        if self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value or self._state == RRTCONNECTSTATE.ETREE_TO_STREE.value:
            # 状態を遷移させる
            self._state_transition()
        # ランダム点を作成する状態では，状態を遷移させない

    def add_node_and_chk_goal(self, start_pos, end_pos, node_pos, near_node):
        """
        ノード追加 + 経路生成の完了確認

        パラメータ
            start_pos(numpy.ndarray): 経路生成の始点
            end_pos(numpy.ndarray): 経路生成の終点
            node_pos(numpy.ndarray): ノード位置
            near_node(int): 親ノード

        戻り値
            is_successful(bool): True/False = 経路生成の完了/未完了
        """
        # 処理結果
        is_successful = False

        # ノードを追加するツリーを選択
        if self._state == RRTCONNECTSTATE.STREE_RAND.value or self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value:
            # 始点ツリーにノードを追加する
            your_tree = self._end_tree
            self._add_node_start_tree(node_pos, near_node)
        else:
            # 終点ツリーにノードを追加する
            your_tree = self._start_tree
            self._add_node_end_tree(node_pos, near_node)

        if self._state == RRTCONNECTSTATE.STREE_TO_ETREE.value or self._state == RRTCONNECTSTATE.ETREE_TO_STREE.value:
            # 始点ツリーまたは終点ツリーにノードを伸ばしている状態
            # 相手ツリーのノード位置を目標点とする
            target_pos = your_tree.nodes[-1, :self._NODE_NEAR_NODE_IDX]

            # 目標点との距離が一定範囲内であるかの確認
            if self._chk_end_pos_dist(node_pos, target_pos):
                # 一定範囲内のため，経路生成の完了
                is_successful = True
        else:
            # 状態を遷移させる
            self._state_transition()

        return is_successful

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

    def fin_planning(self, start_pos, end_pos):
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


    def preparation(self, start_pos, end_pos, interpolation):
        """
        経路生成の準備

        パラメータ
            start_pos(list): 経路生成の始点
            end_pos(list): 経路生成の終点
            interpolation(int): 補間方法 (関節空間/位置空間)
        """
        # 親クラスの処理を実装
        super().preparation(start_pos, end_pos, interpolation)
        # 終点ノードをツリーに追加
        self._add_node_end_tree(end_pos, INITIAL_NODE_NEAR_NODE)
    
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
