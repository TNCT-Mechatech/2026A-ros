# stm32_pos_control_test（この端末向け・複数STM対応）

2026-10-17: [STM設定適用確認 v2](../../docs/stm_config_id.md)へ対応。
各ボードの全輪のsession_id/config_idを確認してから操作を受け付け、STM再起動時は指令を破棄します。

元リポジトリをそのままcloneしたものは
`reference/stm32_board_pid_pos_vel_control_set/` に保存しています。
参照コミットは `78edb8b795f9315d4492b7b9716ea78b12200a4f` です。
このパッケージはその `src/stm32_pos_control_test` をコピーしてカスタマイズしています。
参照用cloneは変更しておらず、`reference/COLCON_IGNORE` でビルド対象から除外しています。

## 起動

Ubuntu 24.04 / ROS 2 Jazzy。ワークスペースのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stm32_pos_control_test
source install/setup.bash
ros2 run stm32_pos_control_test multi_stm32_control
```

ビルド済みなら次の1行でも起動できます。

```bash
./src/stm32_pos_control_test/scripts/run_multi.sh
```

この端末では依存パッケージがビルド済みです。新しい環境ではワークスペースの
依存パッケージも先にビルドしてください。既存の通常ビルドとsymlinkビルドを
同じbuildディレクトリで混在させると、メッセージ生成時にリンク作成が失敗する場合があります。

1台につき1つの独立したROSノードを起動します。3台は同時に通信し、各ノードが
専用のポート・PIDゲイン・送受信バッファ・10 msタイマー・CSVを持ちます。
同じモーター番号、frame IDを別のSTMでも使えます。
1台が切断・終了しても、ほかのノードは通信を継続します。
この方式は指令の同時刻実行を保証するものではありません。

既定の接続先は、この端末で検出した次の3台です。

| 名前 | ST-Link番号 | PIDゲイン |
|---|---|---|
| `wheel`（足回り） | `066EFF565653756687235135` | `~/.ros/natto/pid_gain.yaml` |
| `inject`（注入） | `0671FF505071655067254019` | `~/.ros/natto/inject_pid_gain.yaml` |
| `roll`（ロール） | `0671FF555185754867214745` | `~/.ros/natto/roll_pid_gain.yaml` |

設定は [config/multi_stm32.yaml](config/multi_stm32.yaml) にあります。
USBを挿し直してtty番号が変わっても、`/dev/serial/by-id/` の個体番号で選びます。
同じポートを使う `serial_motor_bridge`、`inject2026A`、`param_checker` は停止してから起動します。

現在の足回り用ファイルには速度ゲインがあり、位置ゲインはありません。
注入・ロール用ファイルがない場合は、受信・`status`・`stop` 用として接続し、
速度・位置指令は拒否します。別の機構のゲインを流用せず、各STM用に調整して保存してください。
ゲインを保存・変更した後は、このテストを再起動します。

## 端末から指令する

各STMの `Ready` 表示とfeedback受信を確認してから入力します。

```text
status                      # 全STMの状態・生カウント・速度・ゲイン有無
wheel vel 0.02              # 足回りSTMのmotor 0へ速度0.02 RPS
wheel vel 2 -0.01           # 足回りSTMのmotor 2へ速度-0.01 RPS
inject pos 0 1000           # 注入STMのmotor 0へ絶対生カウント1000（位置ゲイン必須）
roll pos 2 -500             # ロールSTMのmotor 2へ絶対生カウント-500（位置ゲイン必須）
use inject                  # 以降の省略時接続先を注入に変更
vel 0.01                    # 選択中のSTMの既定motorへ指令
wheel vel all 0.01 0.01 0.01 0.01   # 足回りSTM内の4モーター
all status                  # すべてのSTMへstatus
wheel stop                  # 足回りだけ停止
stop                        # すべてのSTMをDuty 0にして停止
quit                        # すべて停止して終了
```

`all <command>` は各STMに同じ指令を送ります。`wheel pos all ...` の `all` は
そのSTM内の4モーターを指します。`stop`、`quit`、Ctrl+C、入力終了は全STMを停止します。
個別に送ったモーター指令は `stop` または別の指令まで10 ms周期で再送されます。
別のモーターへの入力は、先に指令したモーターを停止しません。
位置指令は**絶対生カウント**です。既定では起動時に原点をリセットしません。
feedback未受信・古い場合は指令を拒否し、動作中に1秒以上受信が途絶えた場合は
そのSTMの全モーターを停止します。

## 接続台数や設定を変える

```bash
# 足回りと注入だけ
ros2 run stm32_pos_control_test multi_stm32_control --boards wheel inject

# 任意のポート、機構別ゲイン、PPR・ギア比・極性を設定
cp src/stm32_pos_control_test/config/multi_stm32.yaml /tmp/my_stm32.yaml
# /tmp/my_stm32.yamlを編集
ros2 run stm32_pos_control_test multi_stm32_control --config /tmp/my_stm32.yaml
```

`boards` に項目を追加すれば接続台数を増やせます。名前はROS名前空間になります。
`parameters` が全STM共通、各board内の値が個別の上書きです。
同じ実デバイスを二重登録した設定や、CSVがゲインファイルを上書きする設定は拒否します。
各STMには、現在の `natto_robot_hardware` と同じframe ID 0〜20、初期化44 byte、
feedback 32 byteのファームウェアが必要です。

既定の速度上限は0.03 RPS。保存済みの速度・位置上限がさらに小さい場合はそちらを使います。
`max_motor_rps: 0.0` は保存済み上限を使う指定です。
上限のない旧形式YAMLでは0.03 RPSを使います。
`encoder_counts_per_revolution = encoder_pulses_per_revolution × 4 × gear_rate` です。
位置調整済みYAMLにエンコーダ校正値がある場合は、その値を使用します。
`inject`・`roll` の初期値は既存の `inject2026A/config/serial_ports.yaml` に合わせています。

## 元と同じ1台用コマンド

```bash
ros2 run stm32_pos_control_test stm32_pos_control_test

ros2 run stm32_pos_control_test stm32_pos_control_test --ros-args \
  -r __ns:=/inject \
  -p serial_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505071655067254019-if02 \
  -p pid_gain_path:=$HOME/.ros/natto/inject_pid_gain.yaml
```

元の `vel`、`pos`、モーター指定、4台一括指定、数値だけの位置指令、`stop`、`help`、
`quit` を維持しています。1台用の既定ゲインパスは、このワークスペースの
`param_checker` に合わせて `~/.ros/natto/pid_gain.yaml`（`ROS_HOME` 設定時はその下）です。
旧形式 `wheelN: {kp, ki, kd}` と、元リポジトリの `schema_version: 2` の
`wheelN.velocity` / `wheelN.position` 形式を読み込めます。
保存済み速度 `kd` は既存の実機ブリッジと同様に0で送信します。

位置ゲインの調整には、この端末に移植済みの次のコマンドを使います。
このテストを終了し、対象STMのエンコーダ設定と速度ゲインを指定してください。

```bash
ros2 launch param_checker param_checker_position.launch.xml \
  serial_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02 \
  gain_output_path:=$HOME/.ros/natto/pid_gain.yaml \
  target_motor_index:=0 position_operation:=tune
```

機構別の校正設定は `serial_config:=/path/to/config.yaml` で指定できます。
速度調整からの詳しい手順は [param_checker/USAGE.md](../param_checker/USAGE.md) を参照してください。

## topicとCSV

| 出力 | 内容 |
|---|---|
| `/wheel/feedback0`〜`3` | 足回りの `debug_msgs/msg/Feedback` |
| `/inject/feedback0`〜`3` | 注入のfeedback |
| `/roll/feedback0`〜`3` | ロールのfeedback |
| `/<board>/encoder_raw_count0`〜`3` | `std_msgs/msg/Int64` の正確な64 bit生カウント |

この環境の既存 `Feedback.encoder_count` はfloat32です。
正確な位置値はInt64 topicまたはCSVの `encoder_raw_count` を使ってください。

```bash
ros2 topic echo /wheel/feedback0
ros2 topic echo /inject/encoder_raw_count0
```

CSVは `~/.ros/stm32_pos_control_test/motor_control_<日時>_pid<PID>.csv` に
プロセス別に保存します（`ROS_HOME` 指定時はその下）。
boardごとの `csv_path` で保存先を変更できます。
`pid_state` は0=停止、1=速度、2=位置。生カウント、指令、速度、出力、Duty、ゲインを記録します。

## 検証

```bash
colcon test --packages-select stm32_pos_control_test --event-handlers console_direct+
colcon test-result --verbose
```

テストでは3台の疑似シリアルSTMを同時に動かし、接続先別のframe送信、ゲイン・極性、
64 bit位置指令とCSV、指令上限、未調整STMの指令拒否、受信タイムアウト、全台停止を確認します。
実機のモーターを動かす検証は含みません。
