# roll2026A

Ros2 Humble

ロール単独で使うノードです。接続先はST-Link識別番号
`0671FF555185754867214745` の `/dev/serial/by-id/` に固定しています。
USBの接続位置や接続順序による設定変更は不要です。

```bash
source install/setup.bash
ros2 run roll2026A main
```

基板交換時などに接続先を変更する場合は `serial_path` パラメータを指定できます。

```bash
ros2 run roll2026A main --ros-args -p serial_path:=/dev/serial/by-id/交換後のデバイス名
```

注入・ロールをまとめて使う場合は `ros2 launch inject2026A inject.launch.xml` を使います。
同じロール用ポートを開くため、統合ノードと単独ノードを同時には起動しないでください。

`Permission denied` が出る場合は、[シリアルポートの権限設定](../natto_robot_hardware/README.md#初回のシリアルポート権限設定)を行ってください。
