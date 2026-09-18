# inject2026A

Ros2 Humble

`main` は1プロセス内で2つのROS 2ノードを実行し、注入機構用とロール機構用の
2台のマイコンへ、それぞれ別のUSBシリアルポートで接続します。

- Joy入力: `inject/joy`（両ノード共通）
- 注入機構フィードバック: `inject/fb_speed_topic`
- ロール機構フィードバック: `roll/fb_speed_topic`

USB接続位置を変更した場合は、`src/main.cpp` の `INJECT_SERIAL_PORT` と
`ROLL_SERIAL_PORT` を実機の `/dev/serial/by-path/...` に合わせてください。
