# param_checker

実機の測定データからPID係数を計算し、既定では `~/.ros/natto/pid_gain.yaml` に保存します。
メカナムの通信ノードも同じファイルを読み込みます。`ROS_HOME` 設定時はその配下の
`natto/pid_gain.yaml` を使用します。

```bash
source install/setup.bash
ros2 run param_checker param_checker
```

出力先を変更する場合:

```bash
ros2 launch param_checker param_checker.launch.xml gain_output_path:=/absolute/path/pid_gain.yaml
```

`ros2 run` の場合は `--ros-args -p gain_output_path:=/absolute/path/pid_gain.yaml` を指定します。
生成中は同じマイコンを使うメカナム通信ノードを停止してください。
測定操作は従来どおり必要です。保存成功時は `PID gains saved:` と保存先を表示します。
親ディレクトリは自動作成し、保存できなければ理由をエラー表示します。

[読み込み側を含む運用手順](../natto_robot_hardware/README.md#pid設定の生成と読み込み)


## STM個別フレーム通信

[通信移行報告](../../docs/stm_serial_migration.md)を参照してください。
起動時、PID gainはInitialization ID0〜3へ送り、その後に現在モードのzero commandを送ります。
JoyごとにInitializationを再送しないため、原点リセットやPID状態を毎回リセットしません。
速度用では `initial_kp/ki/kd` で測定ゲインを指定します。測定後のPID gain YAML/CSV保存は従来どおりです。
`pid=true` はID4〜7の速度目標（rps）、既定の `pid=false` はID12〜15のsigned dutyを使用します。
同一モータに複数種類のcommandを送信せず、停止時もモードとgainを維持します。
Joy入力が `timeout_ms`（既定300ms）途絶えた場合はゼロ指令を送信します。
ID16〜19は個別に保持し、drain後、実際に更新されたモータだけをpublish/VRFTへ記録します。
ID20も登録して受信します。CSVの既存target_rps列はpid=falseでは従来同様に直接出力指令です。
`serial_config` launch引数でPPR・ギア比・極性などのYAMLを指定できます。
車輪ブリッジと同じエンコーダ設定を使用し、校正時は機体に合わせて変更してください。
