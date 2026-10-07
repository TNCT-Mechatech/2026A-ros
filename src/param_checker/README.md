# param_checker

[使い方ガイド（起動・操作・設定・結果確認）](USAGE.md)

速度ゲイン測定と位置ゲイン測定はlaunchを分けています。

```bash
# 速度ゲイン測定（従来の4輪VRFT）
ros2 launch param_checker param_checker_velocity.launch.xml
# 位置Pゲイン測定（保存済み速度PIを使用、対象モータを指定）
ros2 launch param_checker param_checker_position.launch.xml target_motor_index:=0
# 保存済み位置ゲインで実際の位置指令を確認
ros2 launch param_checker param_checker_position.launch.xml \
  target_motor_index:=0 position_operation:=verify
```

位置用は `position_checker` を実行します。位置ゲインの単位は `rps/encoder_count`、
位置指令は64bitの生カウントです。速度・位置のKdと位置Kiは0固定です。
測定は対象モータの最初のfeedbackから30秒間、他の3台はゼロ出力にします。
既存の `wheelN.kp/ki/kd` を保持して `wheelN.position` を追記します。
[位置制御の手順と全オプション](USAGE.md#12-位置制御のゲイン測定と動作確認)を参照してください。

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
