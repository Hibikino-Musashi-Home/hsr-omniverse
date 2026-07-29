# carrobo — カーロボ@Home 実習用 Isaac Sim 環境

<img src="docs/thumbnail.png" width="400" alt="カーロボ実習アリーナ (俯瞰)">

## 使い方

```bash
make build          # イメージをビルド (初回のみ)
make up             # シミュレータ起動
make up TIME=600    # 競技モード: 600秒(シミュ内時間)で自動終了。
                    # 4方向カメラの映像を1本に合成した動画が
                    # recordings/日付_時刻/arena.mp4 に保存される
make tune           # 録画カメラ4台の位置・画角を GUI で見ながら調整する
make down           # 停止
make ros            # ros2 コンテナに入る (ros2 topic list など)
```

## 録画カメラの調整 (`make tune`)

競技モードの4方向カメラの位置・画角は `configs/placement.yaml` の
`arena_cameras:` で決まる。`make tune` で起動すると録画せずにカメラだけ作られ、
Isaac Sim の Viewport でカメラを選んで動かしながら調整できる。

動かすたびに次の2つが書き出される。

- `recordings/tune/arena_cameras.yaml` … そのまま `placement.yaml` に貼れる設定
- `recordings/tune/preview.png` … 実際に録画される 2x2 の絵

設定の書きかたは `configs/placement.yaml` のコメントを参照。
