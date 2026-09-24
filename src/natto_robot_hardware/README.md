# USBシリアルの個体指定

USBの接続位置ではなく、ST-Linkの識別番号を含む `/dev/serial/by-id/` で接続先を指定します。

| 機構 | ST-Link識別番号 | 登録状況 |
| --- | --- | --- |
| メカナム足回り | `066EFF565653756687235135` | 設定済み |
| 注入 | `0671FF505071655067254019` | 設定済み |
| ロール | `0671FF555185754867214745` | 設定済み |

メカナム用のパス:

```text
/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02
```

`config/serial_motor_bridge.yaml`、`serial_motor_bridge` と `param_checker` の
パラメータ初期値、および `natto_nhk2025_sample` のメカナム実機用launchに設定しています。
launchの `serial_path` 引数で指定した値はYAMLより優先されます。
基板交換時はこれらの既定値を更新するか、起動時に `serial_path` を指定してください。

個体の再確認や基板交換時は、制御プログラムを停止して対象の1台だけを接続し、
`ls -l /dev/serial/by-id/` で個体番号を確認してから登録します。
ST-Linkが外付けの場合は、マイコンとの組み合わせも固定してください。

設定後はUSBの接続位置・接続順序を変えた状態とPC再起動後に、
同じ個体へ接続することを確認してください。
メカナムのbridgeは指定した個体が不在ならエラーを出して同じパスへの再接続を試みます。

注入・ロール用は `inject2026A/config/serial_ports.yaml` と `inject2026A` のコードの既定値に登録しています。
`ros2 launch inject2026A inject.launch.xml` で設定ファイルを読み込めます。
ロール単独の `roll2026A` も同じ個体を既定値に指定しています。

## 初回のシリアルポート権限設定

`Opening serial device:` の後に `Permission denied` が出る場合は、
接続先の識別に成功していても、ユーザーにデバイスを開く権限がありません。
このPCのUSBシリアルデバイスは `root:dialout`、権限 `0660` で作成されます。
初回のみ、利用するユーザーを `dialout` グループへ追加してください。

```bash
sudo usermod -aG dialout "$USER"
```

ログアウトしてログインし直すと、新しいグループが反映されます。
現在のターミナルですぐ確認する場合は、次で新しいシェルを開始します。

```bash
newgrp dialout
id -nG
source install/setup.bash
ros2 launch inject2026A inject.launch.xml
```

`id -nG` に `dialout` が含まれることを確認してください。
`newgrp` はそのシェルに反映されるため、他の既存ターミナルはログインし直すまで
古いグループのままです。グループ追加は保存され、USBの再接続・再起動後も有効です。
