# natto_system_monitor

ROS 2 Jazzy / C++の受動監視。既存制御topicへpublishせず、制御・経路探索・自己位置推定・Serial通信内容/周期を変更しない。共有の `serial_watchdog.hpp` は状態判定のみを提供し、タイムアウト時の停止送信は各制御ノードが行う。全体構成の調査は[INVESTIGATION.md](INVESTIGATION.md)。

## 起動

依存（未導入の場合）:

```bash
sudo apt install ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-aggregator ros-jazzy-rosgraph-monitor ros-jazzy-rsl ros-jazzy-rosbag2-storage-mcap ros-jazzy-plotjuggler-ros
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to natto_system_monitor inject2026A natto_holonomic_pure_pursuit natto_launch natto_nhk2025_sample
source install/setup.bash
ros2 launch natto_system_monitor monitor.launch.py
```

既存全体launchからは次の引数を使用する。監視・記録・debug出力の初期値はfalse。

```bash
ros2 launch natto_nhk2025_sample mecanum_auto.launch.xml launch_system_monitor:=true publish_pursuit_debug:=true
ros2 launch natto_nhk2025_sample mecanum_auto_red.launch.xml launch_system_monitor:=true record:=true bag_directory:=/tmp/trials bag_name:=red_01 publish_pursuit_debug:=true
```

buttons方式は全体launchのnavigation_modeに自動追従。監視だけ別起動する場合は`managed_navigation:=true`を渡す。停止しているlaunch_localizationやlaunch_dynamic_auto_controlに対応する監視は全体launchから無効化する。監視を二重起動しない。

Serial機構を使用する試走では`monitor_inject:=true monitor_roll:=true`を付ける。これらは機構を起動する引数ではない。既存の`ros2 launch inject2026A inject.launch.xml`が2 nodeを起動する。単独の`roll2026A`は削除済みのため、rollも`inject2026A`内の`roll_serial_node`を使用する。Mecanum実機bridgeを監視する場合は`monitor_mecanum:=true`。

## Diagnostics

出力は`/diagnostics`、`/diagnostics_agg`、`/diagnostics_toplevel_state`。`diagnostic_aggregator`は[config/diagnostics.yaml](config/diagnostics.yaml)のplugin設定を読み、`/SYSTEM/{Sensors,Localization,Planning,Control,Hardware}`に集約する。

| Health | 値 | 意味 |
|---|---|---|
| OK | 0 | データ正常、または正常な待機/監視無効 |
| WARN | 1 | 低周期、Path遅延、一時write失敗・回復直後 |
| ERROR | 2 | 不正scan/pose/command、明示的planning失敗、device欠落、USB切断、継続write失敗 |
| STALE | 3 | 未受信、timeout、node/publisher消失、状態管理や診断heartbeatの途絶 |

親カテゴリは`ERROR > STALE > WARN > OK`の優先順位。OKとSTALEの混在をERRORへ変換せず、既知の異常と情報不足を区別する。親には単一の動作状態を推測せずUNKNOWNを入れ、各leafの状態を残す。未受信のカテゴリもSTALE。

| 診断項目 | 主なKeyValue |
|---|---|
| Sensors/Front LiDAR, Rear LiDAR | receive_hz, receive_period_sec, age_sec, last_received_ros_sec, source_stamp_sec, frame_id, range_min/max, point_count |
| Localization/MCL | 上記時刻/周期、pose x/y、frame_id、node/publisherの存在 |
| Planning/Path | point_count, path_exists, frame_id, state, request_id、Path受信時刻 |
| Control/Pure Pursuit | vx/vy/wz、周期・時刻・state、buttons方式ではfeedbackを直接観測 |
| Control/ROS Graph expectations | missing、node_count、期待node/publisher/subscriber |
| Hardware/Serial (各機構node) | port_open, port_disconnected, device, last_success_ros_sec/age_sec, write_success_count, write_failure_count, consecutive_write_errors, last_write_errno/result, expected_write_bytes |
| Hardware/{inject,roll,mecanum} heartbeat | 元の診断と最終受信時刻。未受信/途絶はSTALE |
| Hardware/{inject,roll,mecanum} device | 設定したdevice pathの存在。欠落はERROR。open成功の証明ではない |

Healthとは独立した`operational_state`: INITIALIZING（初期計測/計画待ち）、READY（device path確認済み）、RUNNING、IDLE（目標なし/到着後/serial送信待機）、STOPPED（キャンセル/明示的停止）、DISABLED（監視無効）、UNKNOWN（判断不能）。DISABLEDは機能本体を停止する指示ではない。

PathとPure Pursuitは既存状態管理から実行中と判断した場合だけ周期更新を要求する。buttonsのidle/arrived/canceledは古いPathでもOK。failedはERROR。状態管理自体が途絶えた場合はSTALE。sequenceではgoal受信、非空Path、goal_reached、空Pathから判断する。途中参加ではvolatileの過去イベントを復元できないため、監視をロボット起動時から動かす。

LiDARの±inf/範囲外returnは距離未検出として許容。空scan、範囲/角度/時間メタデータ不正、NaN過多はERROR。poseの非有限値・非単位quaternion・空frameはERROR。受信途絶があれば過去の不正データによるERRORよりSTALEを優先する。

## Parameterと設定

設定は起動時読取り専用（diagnostic_updater自身のparameterを除く）。YAML変更後に監視nodeを再起動する。受信周期はmonotonic時計の窓内計測。ROS時刻/元stampも記録するが、元stampの古さと到着周期は別物。再生時は通常、保存済みdiagnosticsを解析する。

| Parameter | 初期値 / 用途 |
|---|---|
| diagnostic_updater.period | 1.0秒、診断publish周期（各node） |
| frequency_window_sec / startup_grace_sec | 2.0 / 3.0秒。初期情報不足もHealthはSTALE |
| scan_max_nan_fraction | 0.5、NaNの許容割合 |
| quaternion_norm_tolerance | 0.01、quaternion二乗normの許容差 |
| front/rear.enabled, .topic, .node, .min_hz, .timeout_sec | true、実topic、node任意、24Hz、0.5秒 |
| localization.enabled/.topic/.node/.min_hz/.timeout_sec | true、MCL出力、MCL node、30Hz、0.5秒 |
| path.enabled/.topic/.node/.min_hz/.timeout_sec/.warn_age_sec | true、方式別Path、A*、0Hz、5.0秒、2.0秒 |
| command.enabled/.topic/.node/.min_hz/.timeout_sec | true、方式別出力、Pure Pursuit、40Hz、0.5秒 |
| managed_navigation | false。trueはbuttons方式 |
| navigation_status_topic / navigation_manager_node | /navigation/status / /navigation/navigation_manager |
| allow_auto_drive_topic / state_timeout_sec | /controller/allow_auto_drive / 1.0秒。buttonsのmanager heartbeat |
| goal_topic / goal_reached_topic | /goal_pose / /control/dynamic/goal_reached、sequence用 |
| graph.expected_nodes / publisher_topics / subscriber_topics | 空string配列。対象topicの型・publisherと有効targetのnodeは常時確認。subscriber期待値では監視node自身を除外 |
| inject/roll/mecanum.enabled, .device, .diagnostic_name, .timeout_sec | false、device/sourceはmonitor.yaml、3.0秒 |
| serial_diagnostics.error_limit | 3、連続失敗でERROR |
| serial_diagnostics.recovery_warn_sec / idle_sec | 2.0 / 1.0秒 |
| Pure Pursuit publish_debug | false。既存計算結果の任意出力 |

変更したtopicに合わせ、monitor YAMLとrecord YAMLの両方を変更する。Serial nodeのserial_pathを変えた場合は対応するinject/roll/mecanum.deviceも合わせる。`monitor_config:=/absolute/monitor.yaml`で切替。launchの`monitor_lidar/localization/navigation/inject/roll/mecanum`は空文字ならYAMLを尊重し、true/falseなら上書き。

## ROS Graph

rosgraph_monitorはJazzyの外部exec_depend。`rosgraph:=false`で起動を省略できる。`/rosgraph`は`rosgraph_monitor_msgs/msg/Graph`。node死活とpublisher/subscriberの接続性を確認し、rosgraph診断はControlに集約。`rosgraph_config`でignore対象を調整する。未使用の可視化出力なども接続性WARNになるため、実際の構成に合わせる。頻度統計のためのRMW差替え/Deadline変更はせず、対象topicの周期は受動監視で得る。

Jazzy rosgraph_monitor 0.2.3の配布バイナリでは`librsl.so`を要求する一方、依存宣言にrslがない場合がある。本packageはrslもexec_dependに含める。`librsl.so: cannot open shared object file`の場合は`sudo apt install ros-jazzy-rsl`を実行する。Graph起動時は監視側でも`graph.monitor_node`（/graph_monitor）と`graph.monitor_topic`（/rosgraph）の存在を検査し、Graph node自体が消えてもSTALEを表示する。

## MCAP記録

```bash
ros2 launch natto_system_monitor monitor.launch.py record:=true bag_directory:=/tmp/trials bag_name:=trial_01
# 記録のみ
ros2 launch natto_system_monitor monitor.launch.py monitor:=false record:=true bag_directory:=/tmp/trials
ros2 bag info /tmp/trials/trial_01
```

Ctrl-Cで正常終了しindex/metadataを書き切る。bag_name省略時は日時。既存ディレクトリへの上書きは禁止。`record_config`でtopic allow-list、`record_qos_config`でQoSを変更する。MCAP + zstd_fast。scan、pose、path、指令、joy、navigation、joint_states、diagnostics、rosgraph、TFを記録し、PointCloud・costmap・画像・particlesは無条件記録しない。起動後に現れるtopicも検出する。

## PlotJuggler / Foxglove

PlotJugglerのROS2/rosbag対応pluginでMCAPを開く。pluginがMCAP読取りに対応しない環境では、隔離したROS_DOMAIN_IDで`ros2 bag play`してROS2 streaming pluginから読む。

| 解析値 | topic / field |
|---|---|
| 自動走行指令 vx/vy/wz | /control/command_velocity.twist.linear.x/y, angular.z |
| Pure Pursuit直接出力（buttons） | /navigation/feedback.command.twist |
| 選択後target velocity | /command_velocity.twist |
| actual velocity（車輪推定） | /localization/wheel_odometry/odometry.twist.twist |
| path error / goal distance / goal yaw error / 内部target speed | /control/pursuit_debug の同名field |
| Localization更新状態 | /localization/current_pose.header.stamp、/diagnosticsのreceive_hz/age_sec |

PursuitDebugは`natto_msgs/msg/PursuitDebug`。path_errorは最近傍**点**までの距離[m]、goal_yaw_errorは絶対値[rad]、target_speedは加速度制限前[m/s]。実際の出力指令とは区別する。headerは対応する指令と同一時刻。制御処理へ戻さないbest_effort出力。通常は`publish_pursuit_debug:=true`を指定した試走のみ使用。

Foxglove Extensionはこの変更では編集していない。ROS側で判定済みの`/diagnostics_agg`からSYSTEMと5カテゴリのlevelを色分けし、leafの`operational_state`を別欄に表示する。詳細KeyValue、Graph拡張で/rosgraph、Plotパネルで数値topicを表示する。今後のFoxglove側作業はlayout/表示設定と、必要なら単純な診断ツリー表示。監視プロセス全体が停止した場合に備え、UI側にも最終メッセージ受信時刻の表示を用意する。

## Serialの限界

write成功はOSへの全バイトwrite成功を意味し、機構の実動作/firmware ACKの証明ではない。診断オブザーバーは短いwriteも失敗として数え、自身では再送しない。fdのopen状態とPOLLHUP/POLLERRを診断時に観測する。

注入・ロール・足回りは、受信タイムアウトやI/Oエラーでポートを閉じ、既定1秒間隔で再接続する。open・設定失敗も例外として処理し、ノードを終了せずERROR診断を継続する。機構ごとの再接続は独立し、`reconnect_attempts` で初回を含む接続試行回数を確認できる。ポートを閉じた時点で診断用fdも無効化する。再接続処理と停止送信は各制御ノードが担当する。

## 変更ファイル

ルートREADMEに手順リンクを追加。新規ファイルは本package全体と`src/message/natto_msgs/msg/PursuitDebug.msg`。
既存変更はinject2026Aの`src/main.cpp`、`include/*/main.hpp`、`SerialBridge/src/LinuxHardwareSerial.hpp`、`CMakeLists.txt`、`package.xml`、Pure Pursuitのcpp/hpp、natto_launchのpackage.xml・natto.launch.xml・components/control.launch.xml、実機exampleのmecanum_auto/red/blue・mecanum_real/open_loop launch。

## 検証

`colcon test --packages-select natto_system_monitor`で内容検査と単調時計・集約優先度を検証する。追加のintegration testはsequence/buttonsの正常・低周期・不正・途絶・復旧・IDLEと集約を検証する。Serial observer単体テストはpartial write/WARN/ERROR/復旧と呼出回数・引数・errno保持を検証する。

機構packageをビルドした後に`python3 src/monitoring/natto_system_monitor/test/test_serial_pty.py`で疑似端末に対するinject/roll/足回りの実行試験を行える。Pure Pursuitをビルド後に`python3 src/monitoring/natto_system_monitor/test/test_pursuit_debug.py`でdebug有効/無効の出力一致を検証できる。DDS domain 194/196/197/198をテスト専用に使う。実機のUSB抜去・負荷を伴う30Hz/50Hz運転は別途試走確認が必要。

既存package名inject2026Aの命名警告は残る。


### この環境での検証結果 (2026-09-27)

以下は単独roll2026A削除前の記録。現在のPTYテストはinject2026A内のinject/roll両nodeを対象とする。

- 変更対象7 package（natto_msgs, natto_system_monitor, inject2026A, roll2026A, natto_holonomic_pure_pursuit, natto_launch, natto_nhk2025_sample）のビルド成功。最終監視C++再ビルドも成功。
- 自動テスト7件成功（監視integration 2、Serial observer 1、内容/時計/集約のunit 4）。errors/failures/skippedはいずれも0。
- inject/roll実行ファイルをPTYに接続する2テスト成功。追加writeなし、成功回数、切断、連続write失敗を確認。
- Pure Pursuitのdebug有効/無効で同じ入力に対する25回分のTwist一致を確認。debugの距離・yaw error・target speedも確認。
- 全体launchの監視引数引渡し、無効対象のOK/DISABLEDを実起動で確認。実機exampleのlaunch引数解析も成功。
- Graphを含むmonitor launch、MCAP保存、正常終了、ros2 bag infoによる読取りを確認。試験bagにはdiagnostics/diagnostics_agg/diagnostics_toplevel_state/rosgraphの35 messageを保存。
- Python/YAML/XML構文検査とgit diff --check成功。

このマシンではros-jazzy-rslが欠けており、sudoがパスワードを要求するためシステムへの導入は未実施。Graph起動試験は公式debを/tmpへ展開し、そのライブラリパスを当該試験プロセスにだけ設定して実施した。通常起動前に `sudo apt install ros-jazzy-rsl` が必要。実機接続・走行での検証は行っていない。

## シリアル受信タイムアウトの検証

注入・ロール・足回りの各ノードは `serial_watchdog.hpp` で受信／指令期限を判定します。
注入・ロールの `Hardware/Serial` 診断にも `watchdog_state`、`feedback_age_sec`、
`command_age_sec` を追加しています。停止・復帰条件と設定は各ノードのREADMEを参照してください。
診断オブザーバー単体は送信しませんが、制御ノードは起動直後・停止中にゼロ指令を周期送信します。

```bash
colcon test --packages-select natto_system_monitor --ctest-args -R 'test_serial_(watchdog|observer)'
# 3パッケージをビルドし、install/setup.bashをsourceしてから実行
/usr/bin/python3 src/monitoring/natto_system_monitor/test/test_serial_pty.py
```

PTYテストは実機デバイスを開かず、3系統で初回未受信、途中途絶、不正パケット、
系統ごとの独立停止、停止中の指令抑止、復帰、指令途絶、起動時未接続、
3系統の抜去・再接続、繰り返し接続時のfd数を確認します。


長時間の通信不良に対する再実行判定は `serial_restart.hpp` にあり、
`test_serial_restart` で回数・時間の両条件、短い復旧の扱い、正常継続による履歴消去を確認します。
PTYテストには実際のlaunchによる再実行も含まれ、短期通信断の除外、長期通信断後のPID変更、
再実行後のニュートラル待ち、指令途絶だけでは再実行しないことを検証します。
再実行は自動launchで既定有効です。閾値と無効化方法は各通信ノードのREADMEを参照してください。
