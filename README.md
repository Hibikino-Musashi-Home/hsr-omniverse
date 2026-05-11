# HSR-Omniverse

NVIDIA Omniverse / Isaac Sim 上で HSR を扱うためのリポジトリ。

> **Note:** この README は **ROS 2 Humble + Isaac Sim 4.5**(現行の開発対象)向けの手順を記載しています。

---

## クイックスタート (ROS 2 Humble + Isaac Sim 4.5、Docker 構成)

### 動作確認済み環境

| 項目 | バージョン |
|---|---|
| OS | Ubuntu 22.04 |
| GPU | NVIDIA RTX 4070 (VRAM 12GB) |
| NVIDIA Driver | 580.142 |
| Docker | 29.x |
| NVIDIA Container Toolkit | latest |
| Isaac Sim (コンテナ内) | 4.5.0 |
| ROS 2 (コンテナ内) | Humble |

推奨スペック: RTX 30 系以降, VRAM 12GB 以上, RAM 32GB 以上, 空きディスク 100GB 以上。

### 1. ホスト環境のセットアップ

Ubuntu 22.04 をインストールし、NVIDIA プロプライエタリドライバを入れます。BIOS で **Secure Boot を無効化**しておく必要があります(プロプライエタリドライバ使用のため)。

```bash
# NVIDIA ドライバ(Isaac Sim 4.5 では 535 以降が必要)
sudo ubuntu-drivers autoinstall
sudo reboot

# 確認
nvidia-smi
```

### 2. Docker と NVIDIA Container Toolkit のインストール

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

### 3. リポジトリのクローン

```bash
git clone --recursive https://github.com/ry0hei-kobayashi/hsr-omniverse.git
cd hsr-omniverse

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

### 4. キャッシュディレクトリ作成と X11 許可

```bash
# キャッシュディレクトリ(docker-compose でマウントされる)
mkdir -p ~/.hsr-omniverse/cache/{kit,ov,pip,glcache,computecache,data}
mkdir -p ~/.hsr-omniverse/logs

# コンテナから GUI を出すための許可(ログインのたびに必要)
xhost +local:root
```

### 5. ビルド

```bash
docker compose -f docker-compose-ros2.yml build
```

初回は数時間かかります(Isaac Sim 4.5 のベースイメージが約 20GB、加えて HSR 関連パッケージの colcon ビルド)。

### 6. 起動

```bash
docker compose -f docker-compose-ros2.yml up
```

Isaac Sim の初回起動は 10〜20 分かかります(シェーダーコンパイル、USD 読み込み)。2 回目以降はマウントしたキャッシュが効くので速くなります。

RViz は ROS 2 スタックの一部として自動で立ち上がりますが、別途立ち上げたい場合は別ターミナルで:

```bash
docker compose -f docker-compose-ros2.yml exec ros2 \
  /ros_entrypoint.sh rviz2 -d /hsr-omniverse.rviz
```

### 7. ROS 2 トピックの確認

別ターミナルから:

```bash
docker compose -f docker-compose-ros2.yml exec ros2 bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
```

### 終了

```bash
# `docker compose up` を実行しているターミナルで Ctrl+C
# または別ターミナルから:
docker compose -f docker-compose-ros2.yml down
```

---

