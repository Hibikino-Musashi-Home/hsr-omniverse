# carrobo — カーロボ@Home 実習用 Isaac Sim 環境

<img src="docs/thumbnail.png" width="400" alt="カーロボ実習アリーナ (俯瞰)">

## 使い方

```bash
make build          # イメージをビルド (初回のみ)
make up             # シミュレータ起動
make up TIME=600    # 競技モード: 600秒(シミュ内時間)で自動終了し、録画を保存
make tune           # 録画カメラ4台の位置・画角を GUI で見ながら調整する
make down           # 停止
make ros            # ros2 コンテナに入る (ros2 topic list など)
```

## 録画の保存先

`make up TIME=<秒>` の競技モードでは、タスク終了時 (指定した時間が来た時) に
4方向カメラの映像を 2x2 に合成した動画が保存される。

```
recordings/<日付_時刻>/arena.mp4      例: recordings/20260729_154230/arena.mp4
```

日付・時刻は起動した時刻。実行のたびに新しいフォルダが作られるので、
過去の録画が上書きされることはない。
