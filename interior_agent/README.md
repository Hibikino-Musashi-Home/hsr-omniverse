# InteriorAgent 連携

[Spatialverse InteriorAgent](https://huggingface.co/datasets/spatialverse/InteriorAgent) のシーンを使って、hsr-omniverse の HSR をリアルな室内環境で動作させるためのスクリプト群です。

## 構成

```
interior_agent/
├── README.md         この文書
├── download.sh       Hugging Face から指定シーンをダウンロード
├── list_scenes.sh    ローカルにあるシーンを一覧表示
└── run.sh            シーンを指定して HSR を起動
```

関連ファイル(リポジトリルート):

- `sample-interior.py` ── InteriorAgent シーン + HSR の最小構成スクリプト
- `docker-compose.dev.yml` ── ボリュームマウント設定

## 前提

- `docker-compose-ros2.yml` でビルド済み
- Hugging Face CLI(`hf` コマンド)がインストール済み
- `docker-compose.dev.yml` に InteriorAgent のマウント設定あり

## クイックスタート

### 1. 環境変数の設定(初回のみ)

リポジトリ直下に `.env` を作成:

```bash
cp .env.example .env
# .env を編集して、INTERIOR_AGENT_PATH を自分のローカルパスに設定
```

例:

```
INTERIOR_AGENT_PATH=/home/<ユーザー名>/datasets/InteriorAgent
```

設定したパスのディレクトリは事前に作っておく:

```bash
mkdir -p ~/datasets/InteriorAgent
```

### 2. シーンをダウンロード

```bash
./interior_agent/download.sh kujiale_0003
```

複数指定もできる:

```bash
./interior_agent/download.sh kujiale_0003 kujiale_0007 kujiale_0012
```

### 3. ローカルにあるシーンを確認

```bash
./interior_agent/list_scenes.sh
```

例:
```
利用可能なシーン (/home/koshun/datasets/InteriorAgent):
  - kujiale_0003 (1.2G)
  - kujiale_0007 (980M)
```

### 4. HSR を InteriorAgent シーンで起動

```bash
./interior_agent/run.sh kujiale_0003
```

このコマンドの中身:
1. `docker compose ... exec isaacsim bash` でコンテナに入る
2. 環境変数 `INTERIOR_AGENT_SCENE=kujiale_0003` をセット
3. `sample-interior.py` を実行

シーン読み込み完了後、HSR が部屋の中に立っているはず。

### 5. 別ターミナルでテレオペ

```bash
docker compose -f docker-compose-ros2.yml -f docker-compose.dev.yml exec ros2 bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/command_velocity_teleop
```

`i` / `,` / `j` / `l` / `k` キーで HSR を操作できる。

## どんなシーンが取得できるか

利用可能なシーン名は Hugging Face の [データセットページ](https://huggingface.co/datasets/spatialverse/InteriorAgent/tree/main) で確認。

`kujiale_XXXX` 形式の名前で、それぞれが 1 つの住宅(リビング + 寝室 + キッチンなど複数の部屋を含む)に対応する。

1 シーンあたり 500MB 〜 数 GB。

## sample-interior.py の主な調整箇所

### HSR の配置位置

シーンによって部屋のレイアウトが違うので、HSR の初期位置は調整が必要:

```python
HSR_TRANSLATION = [0.0, 0.0, 0.0]   # [X, Y, Z]
HSR_YAW = 0.0                        # 向き(ラジアン)
```

座標の調べ方:
1. シーンを起動して GUI で部屋を確認
2. Stage パネルで配置したい場所の付近の Prim を選択
3. Property パネルの `Transform > Translate` から座標を取得
4. `sample-interior.py` の `HSR_TRANSLATION` を編集
5. 再実行

### 照明の強度

InteriorAgent には自前の照明があるが、必要に応じて補助照明を追加できる:

```python
EXTRA_LIGHT_INTENSITY = 3e4   # 0 にすれば消える
```

## トラブルシューティング

### HSR が床に埋まる / 浮く

`HSR_TRANSLATION` の Z 値を調整。シーンによっては床の高さが 0 でないことがある。

### HSR が壁の中に出現する

`HSR_TRANSLATION` の X, Y 値を調整。GUI でクリアな場所の座標を確認してから設定。

### シーン読み込みが終わらない

InteriorAgent はメッシュ・テクスチャが大量なので、初回は 5〜15 分かかる。`docker compose up` のターミナルでログが流れ続けていれば動作中。

### `Warning: ... has corrupted data in primvar 'displayColor'` が大量に出る

InteriorAgent のメッシュデータの仕様問題。表示には影響しないので無視で OK。

### `hf: command not found`

Hugging Face CLI のインストールが必要:

```bash
pip install -U "huggingface_hub[cli]"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

## ライセンス

InteriorAgent は独自の利用規約([InteriorAgent Terms of Use](https://kloudsim-usa-cos.kujiale.com/InteriorAgent/InteriorAgent_Terms_of_Use.pdf))に従って利用すること。研究目的・商用利用の条件を事前に確認。
