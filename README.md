# carrobo — カーロボ@Home 実習用 Isaac Sim 環境

<img src="docs/thumbnail.png" width="400" alt="カーロボ実習アリーナ (俯瞰)">

NVIDIA Omniverse / Isaac Sim 上で HSR を扱う `hsr-omniverse` の実習用ブランチ。
4部屋の TidyUp アリーナと、競技の録画機能を追加している。

> **Note:** ROS 2 Humble + Isaac Sim 4.5

---

## クイックスタート

### 動作確認済み環境

| 項目                     | バージョン                  |
| ------------------------ | --------------------------- |
| OS                       | Ubuntu 22.04                |
| GPU                      | NVIDIA RTX 4070 (VRAM 12GB) |
| NVIDIA Driver            | 580.142                     |
| Docker                   | 29.x                        |
| NVIDIA Container Toolkit | latest                      |
| Isaac Sim (コンテナ内)   | 4.5.0                       |
| ROS 2 (コンテナ内)       | Humble                      |

推奨スペック: RTX 30 系以降, VRAM 12GB 以上, RAM 32GB 以上, 空きディスク 100GB 以上。

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
git clone --recursive -b carrobo \
  https://github.com/Hibikino-Musashi-Home/hsr-omniverse.git carrobo-isaac
cd carrobo-isaac

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

サブモジュールは `usd/hsrb` (HSR の USD) と `usd/wrs_models` (YCB 物体・家具)。
これらが無いとロボットも物体も出ないので、必ず取得しておくこと。

### 3. キャッシュディレクトリ作成と X11 許可

```bash
# キャッシュディレクトリ (docker-compose でマウントされる)
mkdir -p ~/.carrobo-isaac/cache/{kit,ov,pip,glcache,computecache,data}
mkdir -p ~/.carrobo-isaac/logs

xhost local:
```

### 4. ビルド・起動

操作は `Makefile` 経由で統一している。

```bash
make build          # 全イメージをビルド (初回のみ。30分〜1時間程度)
make up             # シミュレータ一式を起動
make up TIME=600    # 競技モード: 600秒(シミュ内時間)で自動終了し、録画を保存
make down           # 停止・コンテナ削除
make ps             # コンテナの状態
make logs           # 全サービスのログを tail
make ros            # ros2 コンテナで bash (ros2 topic list などを叩く場所)
make isaacsim       # isaacsim コンテナで bash

make                # 引数なし → help
```

よく使うオプション:

```bash
make up RVIZ=1                  # RViz2 も起動する (既定はオフ)
make up BASE_DIRECT_DRIVE=0     # 台車を物理車輪駆動に戻す
make up BASE_TRAJ_P_GAIN=1.0    # whole_body の台車追従を弱める
```

Isaac Sim の初回起動は 10〜20 分かかる (シェーダーコンパイル、USD 読み込み)。
2 回目以降はマウントしたキャッシュが効くので速くなる。

### 5. ROS 2 トピックの確認

別ターミナルから:

```bash
make ros
# コンテナ内で:
ros2 topic list
```

`make ros` は `/ros_entrypoint.sh` 経由で bash を起動するので、ROS 環境
(`/opt/ros/humble` と `/ws` の `hsrb_interface` 等) は source 済み。

#### トラブルシューティング: トピックが `/parameter_events` と `/rosout` しか出ない

`ros2 topic list` の結果がこの 2 つだけになることがある。多くの場合トピックが流れて
いないのではなく、**`ros2` の探索デーモン (今あるトピック/ノードを覚えておく裏方プロセス)
が古い「空」の状態をキャッシュしている**ため。Isaac Sim はロードに時間がかかる
(初回はシェーダーコンパイルで 10〜20 分) ので、Isaac Sim がトピックを出し始める前に
デーモンが起動すると「何も無い」と覚えたまま更新されない。

対処は 1 行。デーモンを止めれば次のコマンドで自動的に作り直され、最新の状態を取得する:

```bash
ros2 daemon stop
ros2 topic list   # 再取得
```

正しく流れていれば `/joint_states` (約 30Hz)・`/scan`・`/head_rgbd_sensor/...` (カメラ)・
`/tf` や、ノード `/isaac_sim_hsr` などが見える。確認用:

```bash
ros2 topic hz /joint_states     # 流量 (Hz) を見る
ros2 topic echo /scan --once    # 中身を 1 件だけ見る
```

> **切り分けのヒント:** isaacsim コンテナ自身と ros2 コンテナの両方から `ros2 topic list`
> を比べると、「コンテナ間通信の問題」か「Isaac Sim 側がまだ出していない」かを判別できる。
> なお Isaac Sim が完全に起動し終えてから確認すれば、最初から正しく見えることがほとんど。

> **ROS_DOMAIN_ID** は既定 26 (実習の学生環境に合わせている)。別 PC と通信するときは
> 両方で必ず揃えること。ずれると一切つながらない。

---

## 録画の保存先

`make up TIME=<秒>` の競技モードでは、タスク終了時 (指定した時間が来た時) に
4方向カメラの映像を 2x2 に合成した動画が保存される。

```
recordings/<日付_時刻>/arena.mp4      例: recordings/20260729_154230/arena.mp4
```

日付・時刻は起動した時刻。実行のたびに新しいフォルダが作られるので、
過去の録画が上書きされることはない。

録画カメラの位置・画角は `make tune` で GUI を見ながら調整できる
(結果は `recordings/tune/` に出力。`make tune-apply` で `configs/placement.yaml` に反映)。

---

## 開発モード

シミュレータの Python を手で実行したいとき (ログ・エラーをその端末だけに出したいとき):

```bash
make dev up     # コンテナだけバックグラウンド起動 (シミュレータは自動起動しない)
make dev run    # 同じ端末でシミュレータを実行
make dev down   # 停止
```

`dev` を付けると `scripts/` がディレクトリごと live マウントされるので、
編集がコンテナ再起動なしで反映される。

---

## ディレクトリ構成

```
.
├── assets/        # 設定/リソース (cyclonedds.xml, rviz 設定, joint_limits)
├── configs/       # 実行時設定 (placement.yaml, dressing.yaml, textures/) ※下のリンク参照
├── docs/          # README 用の画像
├── env_docker/    # Dockerfile.* と docker-compose.yml
├── examples/      # 制御指示のサンプルコード (実習用。live マウント)
├── launch/        # ROS 2 launch ファイル (hsr.launch.py)
├── recordings/    # 競技モードの録画出力 (git 管理外)
├── scene_dressing/# 部屋の見た目 (テクスチャ・マテリアル) 関連
├── scripts/       # 実行スクリプト (launch_isaacsim.py, hsr.py, arena_cameras.py 他)
├── usd/           # USD アセット (hsrb/ と wrs_models/ はサブモジュール)
└── worlds/        # .world ファイル (家具・壁の配置)
```

エントリポイント: `scripts/launch_isaacsim.py`。Isaac Sim から HSR を起動するメインスクリプト。

設定まわり (world / 物体・ロボットの配置 / 見た目): [configs/README.md](./configs/README.md)
