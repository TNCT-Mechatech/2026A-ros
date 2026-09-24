# inject2026A

Ros2 Humble

`main` は1プロセス内で2つのROS 2ノードを実行し、注入機構用とロール機構用の
2台のマイコンへ、それぞれ別のUSBシリアルポートで接続します。

- Joy入力: `inject/joy`（両ノード共通）
- 注入機構フィードバック: `inject/fb_speed_topic`
- ロール機構フィードバック: `roll/fb_speed_topic`

接続先は各ノードの `serial_path` パラメータで指定します。
注入用はST-Link識別番号 `0671FF505071655067254019` の `/dev/serial/by-id/` に固定済みです。
ロール用はST-Link識別番号 `0671FF555185754867214745` の `/dev/serial/by-id/` に固定済みです。

設定ファイルを使って起動する場合:

```bash
source install/setup.bash
ros2 launch inject2026A inject.launch.xml
```

`config/serial_ports.yaml` のノード名ごとに接続先を設定します。
別の設定ファイルを使う場合は `serial_config:=/absolute/path/serial_ports.yaml` を指定します。

従来の `ros2 run inject2026A main` も使えます。この場合はコードの既定値を使います。
YAMLを渡す場合は次のように起動します。

```bash
ros2 run inject2026A main --ros-args --params-file src/inject2026A/config/serial_ports.yaml
```

この実行ファイルは注入とロールの両方に接続します。いずれかのポートを開けなければ
プロセスは終了します。起動ログの `Opening serial device:` に対象パスを表示します。
USBポートの変更では設定変更は不要ですが、基板を交換した場合は識別番号の更新が必要です。

`roll2026A` の単独ノードは同じロール用ポートを使うため、この実行ファイルと同時には起動しないでください。

`Permission denied` が出る場合は、[シリアルポートの権限設定](../natto_robot_hardware/README.md#初回のシリアルポート権限設定)を行ってください。
