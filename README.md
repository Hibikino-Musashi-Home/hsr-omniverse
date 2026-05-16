# HSR-Omniverse

NVIDIA Omniverse / Isaac Sim 上で HSR を扱うためのリポジトリ。

> **Note:** **ROS1 noetic or ROS 2 Humble + Isaac Sim 4.5**

---

## クイックスタート (ROS 2 Humble + Isaac Sim 4.5)

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
git clone --recursive https://github.com/ry0hei-kobayashi/hsr-omniverse.git
cd hsr-omniverse

# --recursive を忘れた場合は:
git submodule update --init --recursive
```

### 3. キャッシュディレクトリ作成と X11 許可

```bash
# キャッシュディレクトリ(docker-compose でマウントされる)
mkdir -p ~/.hsr-omniverse/cache/{kit,ov,pip,glcache,computecache,data}
mkdir -p ~/.hsr-omniverse/logs

xhost local:
```

### 4. ビルド

```bash
docker compose -f docker-compose-ros2.yml build
```

### 5. 起動

```bash
docker compose -f docker-compose-ros2.yml up
```

Isaac Sim の初回起動は 10〜20 分かかります(シェーダーコンパイル、USD 読み込み)。2 回目以降はマウントしたキャッシュが効くので速くなります。

### 6. ROS 2 トピックの確認

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
