# carrobo-isaac

カーロボ@Home ロボット実習用の **Isaac Sim シミュレータ環境**。

トヨタ HSR (Human Support Robot) を NVIDIA Isaac Sim 4.5 上で動かし、
ROS 2 (Humble) から操作できる。RoboCup@Home 用リポジトリ
[hsr-omniverse](https://github.com/ry0hei-kobayashi/hsr-omniverse) から
実習に必要な最小限の要素を切り出したもの。

- 部屋: `worlds/carrobo.world` の 1 つだけ
  (4部屋の TidyUp アリーナ。高さ 0.6m のきれいな壁 + 本物の棚・机の USD モデル)
- 物体: YCB オブジェクトのみ (`usd/wrs_models`)
- ロボット: HSR-B のみ
- ナビゲーション (LiDAR / odom / laser_scan_matcher) と
  物体把持 (MoveIt / グリッパ / RGB-D カメラ) の機能はそのまま

### 部屋のレイアウト (worlds/carrobo.world)

外周 8.0 x 8.0 m を田の字に 4 部屋 (各 4 x 4 m の正方形) に仕切った構成。
仕切り壁には幅 1.2m のドア開口があり、全部屋を行き来できる。
外から入る入口 (幅 1.2m) は 1 箇所:
**北の壁の西寄り (引き出しのある Room NW へ, x -2.2〜-1.0)**。

```
   y=+4 ┌──────↓入口──┬──────────────┐
        │ Room NW 片付け1 │ Room NE 探索1  │
        │ 引出し+trofast  開   ソファ      │
        │ 長机+トレイ     口       本棚┃  │
        │ +コンテナ       │              │
   y=0  ├──開口──────┼──────開口──┤
        │ Room SW 片付け2 開  Room SE 探索2│
        │ ┃本棚          口  ○円卓  椅子┐ │
        │ ビン×2   高机   │   長机   ビン  │
   y=-4 └──────────────┴──────────────┘
       x=-4            x=0            x=+4     (壁の高さ 0.6m)
```

- **Room NW (西上)**: 片付け先1。段違い引き出し・trofast×3・長机+トレイ2+コンテナA/B
- **Room SW (西下)**: 片付け先2。ビン×2 (黒/緑)・高い机・本棚
- **Room NE (東上)**: 探索1。本棚・**ソファ** + 床に散らばった YCB 物体
- **Room SE (東下)**: 探索2。長机・緑ビン・円形テーブル・部屋の角に椅子1脚 + 床の YCB 物体
- ソファ・円形テーブル・椅子は `configs/placement.yaml` の `furniture:` セクションで配置
  (ソファ: `usd/sofa/Arnold.usd` を scale 0.7 で約1.8m幅に縮小、
   円形テーブル/椅子: `usd/restaurant/`)。大きさは `scale:` で調整できる
- 壁 (`wall_*`) に接触すると `/undesired_contact_detector/detect` に
  True が出る (競技の Hit 判定と同じ仕組み)
- 4部屋 + ドア通過は SLAM・ナビゲーションの練習にちょうどよい構造
- 床 (木目テクスチャ) は壁の外へ約1.4m 広がった正方形で、当たり判定の床
  (GroundPlane) も同じ大きさ・同じ中心。広げ幅は `scripts/launch_isaacsim.py` の
  `FLOOR_MARGIN` で調整できる。**床の外にはロボットを走らせないこと** (落ちる)

## 動作環境

| 項目 | 要件 |
| --- | --- |
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 30 系以降, VRAM 12GB 以上推奨 |
| RAM | 32GB 以上推奨 |
| ディスク | 空き 100GB 以上 |
| Docker | docker compose v2 + NVIDIA Container Toolkit |

Isaac Sim 4.5.0 と ROS 2 Humble はコンテナ内に入るので、ホストへのインストールは不要。

## セットアップ

### 1. Docker と NVIDIA Container Toolkit のインストール

```bash
# Docker
sudo apt update
sudo apt install -y docker.io docker-compose-v2

# ユーザーを docker グループに追加
sudo gpasswd -a $USER docker
# (一度ログアウト・ログインする。または現在のシェルだけなら `newgrp docker`)

# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# コンテナから GPU が見えるか確認
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### 2. リポジトリのクローン

```bash
git clone --recursive <このリポジトリのURL>
cd carrobo-isaac

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

> **重要:** `usd/hsrb` (HSR 本体) と `usd/wrs_models` (YCB 物体, 約900MB) は
> git submodule。中身が空だとビルド・起動に失敗するので、`ls usd/wrs_models`
> で `ycb_...` フォルダが見えることを確認する。

### 3. キャッシュディレクトリの作成と画面許可

```bash
mkdir -p ~/.carrobo-isaac/cache/{kit,ov,pip,glcache,computecache,data} ~/.carrobo-isaac/logs
xhost +local:
```

### 4. ビルドと起動

```bash
make build   # 初回のみ。30分〜1時間程度
make up      # 起動。初回はシェーダー生成で 10〜20 分かかる
```

Isaac Sim のウィンドウが開き、部屋 (rc26.world) + YCB 物体 + HSR が表示されれば成功。

停止は別端末で:

```bash
make down
```

## 動作確認

別の端末で ros2 コンテナに入り、トピックを確認する:

```bash
make ros                 # ros2 コンテナの bash に入る
ros2 topic list
ros2 topic hz /scan                                    # LiDAR (~10Hz)
ros2 topic hz /head_rgbd_sensor/rgb/image_rect_color   # 頭部RGBカメラ
ros2 topic echo /joint_states --once                   # 関節角度
```

ロボットを動かしてみる (前進):

```bash
ros2 topic pub -r 10 /omni_base_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

シーンを初期状態に戻す:

```bash
ros2 service call /isaac/reset_world std_srvs/srv/Empty
```

## 競技モード (時間制限つき実行 + 4方向カメラ録画)

```bash
make up TIME=600    # 競技時間 600 秒 (シミュレータ内時間)
```

`TIME=<秒>` を付けて起動すると競技モードになる:

- アリーナの外側の高い位置に**観戦カメラが 4 台** (北/南/東/西) 設置され、
  部屋全体を見下ろす映像をシミュレータ内部で録画する
  (「どこに何を片付けたか」を人が目視で採点できる)
- **4 画面を 2x2 に並べた 1 本の動画** `recordings/日付_時刻/arena.mp4` に保存される
  (上段: NORTH | EAST、下段: SOUTH | WEST。各画面にカメラ名入り)
- 動画の左上に **タスク経過時間 (`TASK 03:21 / 10:00`)** が焼き込まれる
- 競技時間 (シミュレータ内時間) が来たら**動画を保存してシミュレータ一式が自動終了**する
- 1/20 シミュ秒 = 1 フレームで書き出すため、動画の再生時間 = 競技時間になる

カメラの高さ・距離・解像度は `configs/placement.yaml` に `arena_cameras:`
セクションを書くと変えられる (詳細は `scripts/arena_cameras.py` の冒頭コメント)。
`TIME` を付けない通常起動ではカメラは作られない (描画も軽いまま)。

## 主要な ROS 2 インタフェース

| 種別 | 名前 | 用途 |
| --- | --- | --- |
| Topic (pub) | `/scan` | LiDAR スキャン (ナビゲーション) |
| Topic (pub) | `/head_rgbd_sensor/rgb/image_rect_color` | 頭部 RGB 画像 (物体認識) |
| Topic (pub) | `/head_rgbd_sensor/depth_registered/image_raw` | 頭部 深度画像 |
| Topic (pub) | `/joint_states` | 全関節の角度 |
| Topic (pub) | `/omni_base_controller/wheel_odom` | オドメトリ (シミュレータ真値) |
| Topic (pub) | `/switched_odom` | オドメトリ (切替後の出力。既定で wheel_odom) |
| TF | `map → odom → base_footprint → 各センサ` | 座標変換 |
| Topic (sub) | `/omni_base_controller/cmd_vel` (Twist) | 台車の速度指令 |
| Action | `follow_joint_trajectory` (アーム/頭部) | 関節軌道の実行 |
| Action | グリッパ (apply_force 等) | 物体の把持 |
| MoveIt | `move_group` ノード | アームの動作計画 |
| Service | `/isaac/reset_world` | シーンを初期配置に戻す |
| Service | `/gazebo/get_model_state` | 物体の位置取得 (Gazebo互換API) |

## 設定の変更

| やりたいこと | 編集するファイル |
| --- | --- |
| ロボットの初期位置・物体の配置を変える | [`configs/placement.yaml`](./configs/placement.yaml) |
| 部屋の見た目 (床・壁・照明) を変える | [`configs/dressing.yaml`](./configs/dressing.yaml) |
| 部屋の間取りそのもの | [`worlds/rc26.world`](./worlds/) |

`configs/` と `worlds/` はコンテナに bind mount されているので、
編集後は `make down && make up` だけで反映される (リビルド不要)。
詳細は [`configs/README.md`](./configs/README.md) を参照。

## 開発モード (コードを編集しながら動かす)

```bash
make dev up     # コンテナだけバックグラウンド起動 (シミュレータは自動起動しない)
make dev run    # 同じ端末でシミュレータを実行 (エラーやログがここに出る)
make dev down   # 停止
```

dev モードでは `scripts/` がディレクトリごとマウントされるため、
Python の編集が `make dev run` のやり直しだけで反映される。

## トラブルシューティング

- **ウィンドウが出ない**: `xhost +local:` を実行したか確認。SSH 越しは不可 (ローカルの画面が必要)。
- **起動が遅い**: 初回はシェーダーコンパイルで 10〜20 分かかる。2 回目以降は
  `~/.carrobo-isaac/cache/` にキャッシュされて速くなる。
- **物体が出ない / `Model not found`**: submodule が空の可能性。
  `git submodule update --init --recursive` を実行。
- **`[placement] WARNING` が出る**: `configs/placement.yaml` の家具名が
  `worlds/rc26.world` の `<name>` と一致していない。
- **数時間つけっぱなしで急に落ちる**: メモリ不足 (OOM) の可能性が高い。
  `docker inspect carrobo-isaac-isaacsim-1 --format '{{.State.OOMKilled}}'` が
  `true` なら確定。Isaac Sim は長時間動かすとメモリ使用量が増えるので、
  使わない時間帯は `make down` で止めるのが確実。RAM 32GB のマシンでは
  スワップを増やしておくとさらに安心:
  ```bash
  sudo fallocate -l 16G /swapfile2 && sudo chmod 600 /swapfile2
  sudo mkswap /swapfile2 && sudo swapon /swapfile2
  # 恒久化するなら /etc/fstab に「/swapfile2 none swap sw 0 0」を追記
  ```
- **おかしくなったら**: `make down` してから `make up` でやり直す。

## リポジトリ構成

```
carrobo-isaac/
├── Makefile                 # make build / up / down などの入り口
├── env_docker/              # Dockerfile と docker-compose 設定
├── scripts/                 # Isaac Sim 側の Python (シーン構築・ROS2ブリッジ)
│   ├── launch_isaacsim.py   #   エントリポイント (部屋・物体・HSRを配置して開始)
│   ├── hsr.py               #   HSR 本体の制御と ROS2 ブリッジ
│   └── object_placement.py  #   placement.yaml に従って物体を配置
├── launch/                  # ROS 2 側の launch (MoveIt, odom, teleop など)
├── configs/                 # 実行時設定 (placement.yaml / dressing.yaml)
├── worlds/rc26.world        # 部屋の定義 (家具・壁の配置)
├── scene_dressing/          # 床・背景・照明を作る Python パッケージ
├── assets/                  # DDS 設定・エントリポイントスクリプト等
└── usd/
    ├── hsrb/                # [submodule] HSR-B の 3D モデル
    └── wrs_models/          # [submodule] YCB 物体 + WRC 家具の 3D モデル
```
