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

## PID設定の生成と読み込み

PID係数は `param_checker` で生成します。既定の保存先とメカナム側の読み込み先は
どちらも `~/.ros/natto/pid_gain.yaml` です。`ROS_HOME` を設定している場合は
`$ROS_HOME/natto/pid_gain.yaml` を使います。両方を同じユーザー・同じ `ROS_HOME` で実行してください。
自動生成ファイルはビルドやソースツリーに依存せず、生成後の再ビルドは不要です。

測定時はメカナムの通信ノードを停止し、従来の `param_checker` の測定操作を行います。
起動しただけで適切な係数になるわけではなく、従来どおり実機の測定データが必要です。

```bash
source install/setup.bash
ros2 run param_checker param_checker
# 測定終了後に「PID gains saved: ...」が表示されたことを確認
ros2 launch natto_robot_hardware serial_motor_bridge.launch.xml
```

通常のメカナム実機用・自動制御用launchも同じ既定ファイルを読み込みます。
ファイルがない場合は、先に `param_checker` の測定と保存を完了してください。

別のファイルを使う場合は、生成側に `gain_output_path`、読み込み側に
`gain_config_path` で同じ絶対パスを指定します。

```bash
ros2 launch param_checker param_checker.launch.xml gain_output_path:=/absolute/path/pid_gain.yaml
# 測定・保存が完了したら起動
ros2 launch natto_nhk2025_sample mecanum_auto_red.launch.xml gain_config_path:=/absolute/path/pid_gain.yaml
```

従来の `ros2 run` では `--ros-args -p gain_output_path:=/absolute/path/pid_gain.yaml`
または `--ros-args -p gain_config_path:=/absolute/path/pid_gain.yaml` で指定できます。
空文字は共通の既定パスを意味します。相対パスは起動したディレクトリ基準で解決されるため、
別々のディレクトリから起動する場合は絶対パスを指定してください。

生成側は保存先の親ディレクトリを自動作成し、ファイルを閉じるまでの書き込み失敗を検出します。
保存失敗時は対象パスと理由を表示し、保存成功とは表示しません。
診断CSVの既定出力先もPIDファイルと同じディレクトリです。
`vrft_csv_path` と `serial_diagnostic_csv_path` の明示指定や空文字による無効化は従来どおりです。

## シリアル通信のタイムアウトと復帰

`serial.update()` が `-1`（完全なフレーム未受信）、`-2`（不正パケット）、
`-3`（未登録ID）でも、10ms周期で受信期限を確認します。
ID16〜19の4モータすべての新しいフィードバックが揃うたびに最終受信時刻を更新します。
送信成功や他IDのフレームでは受信期限を延長しません。
判定には `steady_clock` を使用し、ROS時刻の停止・変更に依存しません。

| パラメータ | 既定値 | 用途 |
|---|---:|---|
| `feedback_timeout_ms` | 200 | 正常フィードバック受信後の受信期限 |
| `startup_feedback_timeout_ms` | 1000 | 接続開始から初回受信までの期限 |
| `timeout_ms` | 300 | 正常な操作・走行指令の受信期限 |
| `reconnect_interval_ms` | 1000 | ポート切断後／接続失敗後の再試行間隔 |
| `feedback_recovery_count` | 3 | 動作許可前に必要な正常フィードバックの受信回数 |

すべて正の値を指定します。受信間隔が期限以上空いた場合、復帰用の受信回数はリセットされます。
接続直後・復帰待ち・指令タイムアウト中は目標値を全てゼロにし、接続中は10ms周期で停止指令を再送します。
この間に非ゼロ指令が来ても送信せず、以前の指令を保持して再実行することもありません。
動作開始・再開には正常受信が指定回数続いた後、全モーターの指令がゼロになる
操作（Joyの中立／ゼロ速度指令）が必要です。その後の新しい非ゼロ指令で動作します。
指令入力は `timeout_ms` より短い周期で継続送信してください。

`/diagnostics` の `watchdog_state`、`feedback_age_sec`、`command_age_sec` で状態を確認できます。
初回未受信の受信経過時間は `-1`、受信タイムアウトはERROR、受信待ち・復帰待ち・指令途絶はWARNです。
USB切断やPC停止時は停止指令が届かないため、マイコン側にも指令受信ウォッチドッグが必要です。

受信タイムアウト（初回未受信も含む）ではゼロ指令の送信を試みてポートを閉じ、
`reconnect_interval_ms` 後から接続を再試行します。read/writeエラーや短いwriteでも再接続へ移行します。
ポートを開く／設定する処理に失敗してもプロセスを終了せず、同じ間隔で再試行を続けます。
各回で `serial_path` を開き直すので、USB再接続後にtty名が変わっても、同じ `/dev/serial/by-id/` に戻れば接続できます。
再接続時は送受信バッファと動作許可を初期化し、ゼロ指令から開始します。
正常受信が指定回数続き、その後ゼロ／中立指令を受けるまで非ゼロ指令を送信しません。
指令だけが途絶えた場合はポートを閉じず停止を維持します。各機構の再接続は独立しています。
`/diagnostics` の `reconnect_attempts` は初回接続を含む接続試行回数です。

これらのノードはSerialBridgeのLinux用ソースをcolconで直接ビルドします。
既存の `bin/libSerialBridge.a` の手動再生成は不要です。


### 長時間の通信不良からの自動再実行

`mecanum_auto.launch.xml` と赤・青版では `auto_serial_restart:=true` が既定です。
従来の即時停止・ポート再接続を続けても復旧しない場合、通信プロセスを終了し、
通信不良が1秒続いたらlaunchが追加待機なしで再実行します。`auto_serial_restart:=false` でこの追加処理を無効化できます。
通常の単体起動では無効です。単体で `auto_restart=true` を指定する場合は、launch側にも
`respawn=true` が必要です。

- `restart_failure_count: 1`: 不良状態の最低検出回数。初回検出で回数条件を満たします。
- `restart_timeout_ms: 1000`: 最初の不良判定からの最低経過時間。回数と時間の両方が必要。
- `restart_recovery_ms: 3000`: 履歴を消すために必要な正常通信の継続時間。

正常受信が安定するまで同じ不良期間として扱うため、ポートを開けただけ、あるいは
一瞬だけ受信できた場合には履歴を消しません。指令途絶やニュートラル待ちだけでは再起動しません。
再実行後も正常受信とニュートラル指令が必要で、以前の動作指令は引き継ぎません。
注入・ロールは同一プロセスなので、どちらかの長時間不良で両方が再実行されます。
足回りは別プロセスです。通信が戻らない間は1秒の判定期間ごとに再試行します（プロセスの終了・起動時間は別途かかります）。


## STM個別フレーム通信

[通信移行の調査・設定・検証報告](../../docs/stm_serial_migration.md)を参照してください。
固定IDとABIは `include/natto_robot_hardware/serial_protocol.hpp` に集約しています。
接続・再接続時はID0〜3の初期化後、現在のモードで4モータへゼロ指令を送信します。
速度モードはID4〜7（rps）、open-loopはID12〜15（signed duty）です。

`encoder_polarities=-1` はSTMの `set_direction=true` に変換します。
受信値へROS側で `encoder_polarities` を再度掛けません。`wheel_polarities` は
送信指令とJointStateの速度・位置の座標変換に使います。debugの `feedback0`〜`feedback3` は
STMのモーター座標のまま公開し、`target_rps` と `current_rps` を同じ符号で比較できます。

メカナムの既定値は、FL/FR/RL/RRの順で `wheel_polarities=[-1,1,-1,1]`、
`encoder_polarities=[-1,-1,-1,-1]` です。以前の全輪 `set_direction=true` と
速度調整用YAMLに合わせ、STM内の正の出力に対して測定速度も正になるよう設定します。
FR/RRを `encoder_polarities=1` にすると、逆符号の測定値でPIDが誤差を増幅し、
逆回転・暴走する可能性があります。左右の車輪取り付け方向は `wheel_polarities` で変換します。
反映にはbridgeの再ビルドとlaunchの再起動が必要です。初回は車輪を浮かせて低速で確認し、
`feedback1`（FR）・`feedback3`（RR）の目標速度と測定速度が同符号になることを確認してください。
位置は `encoder_raw_count / (encoder_ppr * 4 * gear_rate) * 2π` です。
`max_feedback_dt_sec` は旧launch互換のため受理しますが、raw countには使用しません。
速度・位置はwheel軸であり、STMの `target_motor_rps` というフィールド名でもCPRはgear_rate込みです。

`encoder_ppr`、`gear_rate`、`max_motor_rps`、`pos_kp/ki/kd`、原点リセット設定は
4要素のROSパラメータです。YAMLのPPR・ギア比はSTM既定値であり機体の校正値ではありません。
`pid_state` はモードから決定し、velocity PIDのkp/kiは従来のPID gain YAMLから送ります。
kdは従来の実機運用方針に合わせて常に0を送信し、YAMLの保存値は維持します。
スイッチは `/switch_feedback` (`std_msgs/UInt8MultiArray`, motor0〜3、0/1) に公開します。
`invalid_packets`、`unregistered_frames`、`drain_budget_hits` はdiagnosticsで確認できます。

通常指令は `min(timeout_ms, 300ms)` より短い周期で継続してください。
指令途絶はROSとSTMの二重停止になり、停止送信は現在のモードを維持します。
STMの通信タイムアウト処理自体はpid=1・brakeへ切り替えるため、実機で復帰挙動を確認してください。
