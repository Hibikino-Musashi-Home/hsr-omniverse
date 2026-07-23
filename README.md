# carrobo — カーロボ@Home 実習用 Isaac Sim 環境

<img src="docs/thumbnail.png" width="400" alt="カーロボ実習アリーナ (俯瞰)">

## 使い方

```bash
make build          # イメージをビルド (初回のみ)
make up             # シミュレータ起動
make up TIME=600    # 競技モード: 600秒(シミュ内時間)で自動終了。
                    # 4方向カメラの映像を1本に合成した動画が
                    # recordings/日付_時刻/arena.mp4 に保存される
make down           # 停止
make ros            # ros2 コンテナに入る (ros2 topic list など)
```
