# param_checker の使い方（ROS 2 Jazzy）

速度用の `param_checker` は、実測feedbackからVRFTでゲインを求めます。
既定は足回り4モータのPID測定、`target_motor_index=0〜3`では選択した1台のPI測定です。
足回りモードではPID YAML・測定CSV・シリアル診断CSVを保存します。
位置用の `position_checker` は、保存済み速度PIと生エンコーダカウントを使って位置Pゲインを求めます。
ROSノード名はどちらも `/param_check` です。1つずつ起動してください。
[速度用オプション一覧](#11-オプション一覧)と
[位置用の手順・オプション一覧](#12-位置制御のゲイン測定と動作確認)を参照してください。

雑巾ラックは[単体調整の手順](#13-雑巾ラックなど機構モーター1台の調整)を使ってください。

以下の1〜11節は既定の足回りモードの説明です。
**速度用はシリアル接続・初期化が成功すると測定がすぐ始まります。開始ボタンや待機時間はありません。**
既定では10秒間測定し、gain算出・ファイル保存・ゼロ指令送信の後に自動終了します。

## 1. 測定前の準備

- 同じSTMポートを使う `serial_motor_bridge` と、そのノードを自動再起動するlaunchを停止します。
- 車輪を動かせる測定環境を準備します。コントローラの軸入力が直接モータ指令になります。
- STMが新通信仕様のID0〜20をすべて登録できることを確認します。
  指定SerialBridge参照版にある登録上限10の問題は[通信移行報告](../../docs/stm_serial_migration.md)を参照してください。
- `encoder_ppr`、`gear_rate`、`encoder_polarities` を実機と合わせます。
  [同梱設定](config/serial_protocol.yaml)の現在の値はPPR=2048、全輪gear_rate=245/6、
  encoder_polarities=-1です。実機に合わせて確認・変更してください。

ビルドする場合はリポジトリルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to param_checker
source install/setup.bash
```

前回の通信移行で作成した `/tmp` の検証用overlayを使う場合は、各端末で
`source /tmp/ros-stm-install/setup.bash` を実行します。
通常の `install/setup.bash` にはその検証バイナリが反映されていない場合があります。

## 2. コントローラを先に起動する

入力topicは `/controller/joy` (`sensor_msgs/msg/Joy`) です。
既にこのtopicを配信していれば、新しいJoyノードは不要です。
単独で起動する場合の例：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node --ros-args \
  -r joy:=/controller/joy \
  -p autorepeat_rate:=50.0
```

別端末で、軸の番号と配信周期を確認します。

```bash
ros2 topic echo /controller/joy --once
ros2 topic hz /controller/joy
```

スティックを保持している間もJoyを継続送信してください。
入力が既定300ms途絶えるとゼロ指令になります。上の例の50Hzは20ms間隔です。

| 操作成分 | 既定のJoy軸 | ノード内の処理 |
|---|---|---|
| x | `axes[0]` | そのまま使用 |
| y | `axes[1]` | 符号を反転して使用 |
| turn | `axes[2]` | そのまま使用 |

物理的にどのスティックへ対応するかはコントローラとJoyドライバに依存します。
`axis_x`、`axis_y`、`axis_turn` で変更できます。ボタンは測定開始・許可に使いません。

## 3. 保存先を分けて起動する

通常運転用のPIDファイルへ直接上書きせず、測定ごとの保存先を指定する例です。
リポジトリルートで設定をコピーし、実機のPPR・ギア比・極性を編集してから使います。

```bash
cp src/param_checker/config/serial_protocol.yaml /tmp/param_checker_measurement.yaml
```

設定が準備できたら、コントローラを中立にして起動します。
シリアルポートは使用するSTMの `/dev/serial/by-id/` パスに合わせてください。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

measurement_dir="$HOME/.ros/natto/measurements/run01"
mkdir -p "$measurement_dir"

ros2 launch param_checker param_checker_velocity.launch.xml \
  serial_config:=/tmp/param_checker_measurement.yaml \
  serial_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02 \
  gain_output_path:="$measurement_dir/pid_gain.yaml"
```

`run01` は測定ごとに変えてください。同じパスを指定すると既存ファイルを上書きします。
CSVも既定ではPID YAMLと同じディレクトリへ保存されます。

launch引数は次の3つです。その他のROSパラメータは `serial_config` のYAMLに記載します。
従来の `param_checker.launch.xml` も同じ速度用ノードを起動します。

| launch引数 | 用途 |
|---|---|
| `serial_path` | STMのシリアルポート |
| `gain_output_path` | PID YAMLの出力先。空文字なら既定保存先 |
| `serial_config` | ROSパラメータYAML。省略時は同梱 `config/serial_protocol.yaml` |

`serial_path` と `gain_output_path` はlaunch引数の値が設定YAMLより優先されます。
特に `gain_output_path` をYAMLだけで変更しても、launch側の既定空文字で上書きされるため、
上の例のようにlaunch引数で指定してください。

## 4. 測定中の操作と終了

1. `VRFT measurement started:` と出力先のログを確認します。この時点から時間計測が始まっています。
2. 残り秒数の表示中に、許容できる範囲で前後・左右・旋回の入力を変化させます。
   4モータについて速度と実際のdutyが変化するデータを取得します。
3. 中立だけのデータや同じ入力だけではgain算出が失敗する場合があります。
4. 既定10秒後に診断が始まり、最後に`wheel0`〜`wheel3`のkp/ki/kdと判定が一覧表で表示されます。
5. `PID gains saved:`、`VRFT CSV written:`、`Serial diagnostic CSV written:` と
   `Diagnosis completed. Shutting down param_checker.` を確認します。

測定時間はノード起動後の経過時間で、最初のJoyやfeedbackから数える時間ではありません。
測定中は更新されたモータのfeedbackだけを記録します。

途中で `Ctrl+C` を押すとゼロ指令の送信を試みて終了します。
測定終了時に保存する実装なので、**途中終了では今回のPID YAML・CSVは保存されません。**
再測定する場合はノードを起動し直します。

## 5. Duty測定と速度測定

| `pid` | 動作 | シリアル指令 | `max_wheel_rps` |
|---|---|---|---|
| `false`（既定） | PIDを使わず出力を指定 | ID12〜15のsigned duty | dutyの制限には使われない |
| `true` | 初期PID gainで速度を制御 | ID4〜7のrps | 4輪の速度指令の最大絶対値 |

既定のduty測定では、メカナムの各輪の合成値を `[-1,+1]` にclampして送信します。
**`max_wheel_rps` を小さくしてもduty出力は小さくなりません。**
速度ブリッジの `open_loop_output_limit` やslew制限は、このノードにはありません。
Duty測定は小さな軸入力から始めてください。

速度測定では4輪の合成値を共通の最大値で正規化し、`max_wheel_rps` を掛けて送信します。
操作による合成式は以下です。符号は実装内で適用済みです。

```text
x    = Joy.axes[axis_x]
y    = -Joy.axes[axis_y]
turn = Joy.axes[axis_turn]

motor0 = -(y + x + turn)
motor1 =   y - x - turn
motor2 = -(y - x + turn)
motor3 =   y + x - turn
```

このノードに `wheel_polarities` パラメータはありません。
`encoder_polarities` はSTMのエンコーダ補正用で、指令の符号を変える設定ではありません。

### kdの扱い

`initial_kd` は既定 `0.0` です。現在のkd無効運用では0のまま使います。
`pid=true` では `initial_kp/ki/kd` をSTMのInitializationへ送信します。
`pid=false` では速度PIDが無効なので、これらのgainはduty指令へ作用しません。

VRFTはkp/ki/kdの3係数を計算してYAMLへ保存します。
**通常運転用 `serial_motor_bridge` は、YAMLのkdが非ゼロでも `vel_kd=0` を送信します。**
param_checker自体は `initial_kd` を指定できる実装なので、0固定の通常ブリッジと区別してください。

## 6. 設定を変更する

`/tmp/param_checker_measurement.yaml` の `param_check.ros__parameters` 内に設定を追加します。
同梱のエンコーダ設定を残したまま、例えば測定設定を次のように指定できます。

```yaml
param_check:
  ros__parameters:
    pid: true
    initial_kp: 0.001
    initial_ki: 1.8
    initial_kd: 0.0
    max_wheel_rps: 1.5
    measurement_seconds: 10.0
    response_time_seconds: 0.3
    sample_time_seconds: 0.01
    joy_deadband: 0.05
    axis_x: 0
    axis_y: 1
    axis_turn: 2
    timeout_ms: 300
```

これは設定項目の例であり、実機に適したgain・速度を保証する値ではありません。
パラメータは起動時に読み込むため、変更後はノードを再起動してください。
測定中の `ros2 param set` で制御値を変更する処理は実装していません。

全パラメータの型・既定値は[オプション一覧](#11-オプション一覧)を参照してください。
`sample_time_seconds` を変更してもノードの10ms timerやSTMの周期は変更されません。
エンコーダ・初期化設定の配列はmotor0〜3の4要素です。
CPRは `encoder_ppr × 4 × gear_rate` で、符号補正はSTM側へ集約しています。

### ros2 runで直接指定する場合

この方法ではlaunchを使わず、同梱YAMLも自動では読み込みません。
必要な設定を `--params-file` で明示します。CLIの `-p` は後ろに置いて設定値を上書きします。

```bash
ros2 run param_checker param_checker --ros-args \
  --params-file /tmp/param_checker_measurement.yaml \
  -p serial_path:=/dev/ttyACM0 \
  -p gain_output_path:=/tmp/param_checker_run01/pid_gain.yaml \
  -p initial_kd:=0.0 \
  -p measurement_seconds:=20.0
```

## 7. 保存ファイルと結果の確認

| ファイル | 内容 |
|---|---|
| `pid_gain.yaml` | wheel0〜3のkp/ki/kd |
| `vrft_param_checker.csv` | 各モータの入力duty・出力rps・時刻・VRFTの回帰情報・算出gain |
| `vrft_param_checker_serial.csv` | timer単位の受信結果・feedback更新有無・command送信数 |

`gain_output_path` 未指定時の保存先は以下です。

- `ROS_HOME` が設定されている場合：`$ROS_HOME/natto/pid_gain.yaml`
- それ以外：`$HOME/.ros/natto/pid_gain.yaml`

PID YAMLの親ディレクトリは自動作成されます。
CSVだけを別ディレクトリへ指定する場合は、その親ディレクトリを事前に作成してください。
相対パスの指定は起動時の作業ディレクトリを基準にするため、絶対パスを使うと確実です。

最後の表示例（数値は説明用）：

```text
PIDゲインの決定値 (VRFT)
Motor    |             kp |             ki |             kd | 判定
----------------------------------------------------------------------------
wheel0   |       0.001000 |       1.800000 |       0.000000 | 算出
wheel1   |       0.001200 |       1.750000 |       0.000000 | 算出
wheel2   |       0.000000 |       0.000000 |       0.000000 | 算出失敗 / データ不足
wheel3   |      -0.001000 |       1.800000 |       0.000000 | 負のゲイン（極性を確認）
----------------------------------------------------------------------------
PID YAML: /path/to/pid_gain.yaml （保存済み）
kd列は計算・保存値です。通常運転の車輪ブリッジは vel_kd=0 を送信します。
```

通常は小数6桁で揃え、極端に小さい・大きいgainは指数表記で表示します。
「算出」は計算結果が得られたことを示し、実機での制御の安定性を保証する判定ではありません。

PID YAMLには以下の形式でwheel0〜3が保存されます。

```yaml
wheel0:
  kp: 0.001
  ki: 1.8
  kd: 0.0
# wheel1、wheel2、wheel3も同じ構造
```

上の数値は形式の例で、算出結果ではありません。
保存成功はgainの妥当性を保証しません。結果表の「判定」欄で全gainが0、負値、非有限値も確認してください。
**算出失敗で0になったgainもファイルへ保存される実装**なので、結果を確認してから通常運転へ適用します。

CSVで見る主な列：

| 測定CSVの列 | 確認内容 |
|---|---|
| `wheel` / `sample_index` | モータ番号とサンプル数 |
| `elapsed_sec` / `sample_dt_sec` | 実際の受信間隔とデータ欠落 |
| `target_rps` | `pid=true`ではrps、`pid=false`ではsigned duty指令 |
| `input_duty` | feedbackのdutyとdirから復元した符号付き実出力 |
| `output_rps` | 実測速度 |
| `used_for_regression` | そのサンプルをgain算出へ使用したか |
| `result_kp` / `result_ki` / `result_kd` | モータごとの算出gain |

`target_rps` は指令値です。VRFTに使う入力は実出力の `input_duty` です。

通信診断CSVの `serial_update_result` は0=正常更新、-1=完全なframeなし、
-2=不正packet、-3=未登録IDです。正常バッチ末尾のdrain終了は正常更新を上書きしません。
混在エラーがあれば、そのバッチのエラー結果が残る場合があります。
`feedback_updated=1` が測定中に継続して記録されているか確認してください。

## 8. Topicで確認する

測定中のノード・入力・モータfeedbackは別端末から確認できます。

```bash
ros2 node info /param_check
ros2 topic echo /feedback0
ros2 topic hz /feedback0
```

| Topic | 型 | 内容 |
|---|---|---|
| `/controller/joy` | `sensor_msgs/msg/Joy` | 操作入力 |
| `/feedback0`〜`/feedback3` | `debug_msgs/msg/Feedback` | 実測rps、duty、dir、encoder_count、指令値等 |
| `/pid_gain0`〜`/pid_gain3` | `debug_msgs/msg/PidGain` | 診断完了時に1回publishする算出gain |

gain topicは通常のQoSで1回publishされ、ノードもすぐ終了します。
終了後にtopic echoを始めても結果を取得できないため、保存YAMLを確認してください。
ID20のswitch feedbackは受信しますが、param_checkerにはswitch用公開topicはありません。
namespaceやremapを指定した場合はtopic名も変わります。

## 9. 通常運転へ適用する

測定完了後にJoy入力を中立に戻し、param_checkerの終了を確認します。
結果を点検したPID YAMLを、車輪ブリッジへ明示指定して起動できます。

```bash
ros2 launch natto_robot_hardware serial_motor_bridge.launch.xml \
  gain_config_path:="$HOME/.ros/natto/measurements/run01/pid_gain.yaml"
```

前の測定用 `measurement_dir` と同じ保存先を指定してください。
通常運転の `serial_motor_bridge` はkp/kiを読み、kdは0を送信します。
PIDファイルは起動時に読み込むため、すでに動いているブリッジへ自動反映はされません。
[通常運転側の説明](../natto_robot_hardware/README.md#pid設定の生成と読み込み)も参照してください。

## 10. 困ったとき

| 症状・ログ | 確認すること |
|---|---|
| `param_check error:` / open serial port失敗 | port名、接続、アクセス権、他プロセスによるポート使用 |
| スティックを保持すると途中で止まる | Joyのautorepeat、topic名、`timeout_ms`より短い配信周期 |
| 操作しても動かない | 軸番号、deadband、測定時間が終了していないか、STMの全ID登録 |
| 速度が逆 / 表に「負のゲイン」 | 配線、上記指令式、エンコーダ極性。受信側で二重反転しない |
| 全gainが0 / data was insufficient | 各モータのfeedback、速度と出力の変化、サンプル数、正のresponse_time |
| CSVの `feedback_updated` が常に0 | STM版・baud 230400・ID16〜19の登録・packet ABI |
| `-2` / `-3` が続く | ROSとSTMのprotocol、ID、packetサイズ、登録上限 |
| PID/CSV保存失敗 | 出力先のアクセス権、ディスク容量、CSV出力先の親ディレクトリ |

現行param_checkerには、車輪ブリッジのfeedback timeoutによる動作禁止・neutral復帰待ち・
自動reconnect・auto restart・`/diagnostics`公開はありません。
feedbackが来なくてもJoyの指令は送信され得るため、測定前に返信を確認してください。
通信エラー時は測定を中断し、接続を確認して起動し直します。STM側の300ms timeoutも停止を担います。
プロセスは例外や保存失敗時にも終了コード0になる場合があるため、ログと保存ファイルで成否を確認します。

## 11. オプション一覧

### Launch引数（3項目）

`ros2 launch param_checker param_checker.launch.xml 名前:=値` で指定します。
すべて省略可能です。`serial_path` と `gain_output_path` はlaunch引数がYAMLより優先されます。

| 引数 | 型 | 既定値 | 用途・指定例 |
|---|---|---|---|
| `serial_path` | string | 下記の既定STMポート | 接続先。`serial_path:=/dev/ttyACM0` |
| `gain_output_path` | string | 空文字 | PID YAML出力先。`gain_output_path:=/tmp/run01/pid_gain.yaml` |
| `serial_config` | string | パッケージ内 `config/serial_protocol.yaml` | 設定YAML。`serial_config:=/tmp/param_checker_measurement.yaml` |

既定STMポート：

```text
/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02
```

### 速度用ROSパラメータ（全28項目）

YAMLでは `param_check.ros__parameters` の下へ記載します。
CLIでは `ros2 run param_checker param_checker --ros-args -p 名前:=値` で指定します。
以下の既定値はYAMLを読み込まない場合のコード既定値です。
同梱YAMLで異なる動作になる項目は表の説明に記載しています。

#### 接続・保存先（4項目）

| パラメータ | 型 | コード既定値 | 用途 |
|---|---|---|---|
| `serial_path` | string | 上記の既定STMポート | シリアル接続先。通信速度は230400固定 |
| `gain_output_path` | string | 空文字 | PID YAML保存先。空なら `$ROS_HOME/natto/pid_gain.yaml` または `$HOME/.ros/natto/pid_gain.yaml` |
| `vrft_csv_path` | string | PID YAMLの親ディレクトリ内 `vrft_param_checker.csv` | 測定CSV。空文字で出力を無効化 |
| `serial_diagnostic_csv_path` | string | PID YAMLの親ディレクトリ内 `vrft_param_checker_serial.csv` | 通信診断CSV。空文字で出力を無効化 |

CSV出力を無効にするCLI例。引用符を含む値をROSへ渡すため、外側のシングルクォートも必要です。

```bash
ros2 run param_checker param_checker --ros-args \
  --params-file /tmp/param_checker_measurement.yaml \
  -p 'vrft_csv_path:=""' \
  -p 'serial_diagnostic_csv_path:=""'
```

#### 測定・操作（14項目）

| パラメータ | 型 | コード既定値 | 用途・適用条件 |
|---|---|---|---|
| `pid` | bool | `false` | false=Duty測定、true=速度PID測定 |
| `initial_kp` | double | `0.001` | 初期速度PIDのkp。4モータ共通。有限値 |
| `initial_ki` | double | `1.8` | 初期速度PIDのki。4モータ共通。有限値 |
| `initial_kd` | double | `0.0` | 初期速度PIDのkd。現在のkd無効運用では0を使用。有限値 |
| `max_wheel_rps` | double | `1.5` | pid=trueの速度指令の最大絶対値。正の値を使用。Dutyの制限には使わない |
| `measurement_seconds` | double | `10.0` | 起動後の測定時間（秒）。正の値を使用 |
| `response_time_seconds` | double | `0.3` | VRFTの目標応答モデルの時定数（秒）。正の値を使用 |
| `sample_time_seconds` | double | `0.01` | VRFTのサンプル時間設定（秒）。現行の演算は実受信時刻差を使用し、timer周期は変更しない |
| `joy_deadband` | double | `0.05` | 各軸の絶対値がこの値未満なら0 |
| `axis_x` | integer | `0` | xに使うJoy軸番号（0始まり） |
| `axis_y` | integer | `1` | yに使うJoy軸番号（ノード内で符号反転） |
| `axis_turn` | integer | `2` | 旋回に使うJoy軸番号 |
| `timeout_ms` | integer | `300` | Joy途絶時にゼロ指令を送る期限（ms）。正の整数 |
| `stop_mode` | bool | `false` | Initializationへ送るSTM停止モード設定 |

速度測定のCLI指定例：

```bash
ros2 run param_checker param_checker --ros-args \
  --params-file /tmp/param_checker_measurement.yaml \
  -p pid:=true \
  -p initial_kp:=0.001 \
  -p initial_ki:=1.8 \
  -p initial_kd:=0.0 \
  -p max_wheel_rps:=1.0 \
  -p measurement_seconds:=20.0
```

#### STM初期化（10項目）

配列はすべてmotor0、1、2、3の順に4要素です。
`double[]` は小数点を付けて記載します（例：`[0.0, 0.0, 0.0, 0.0]`）。

| パラメータ | 型 | コード既定値 | 用途・制約 |
|---|---|---|---|
| `encoder_ppr` | integer[] | `[2048, 2048, 2048, 2048]` | Encoder PPR。各値1〜65535 |
| `gear_rate` | double[] | `[71.0, 245/6, 245/6, 245/6]` 相当 | CPR計算用ギア比。各値は有限かつ正 |
| `encoder_polarities` | integer[] | `[-1, 1, -1, 1]` | 各値±1。-1をSTM set_direction=trueへ変換 |
| `max_motor_rps` | double[] | `[max_wheel_rps, max_wheel_rps, max_wheel_rps, max_wheel_rps]` | STM位置制御用最大速度。各値は有限かつ正。同梱YAMLは `[1.5, 1.5, 1.5, 1.5]` 固定 |
| `pos_kp` | double[] | `[0.0, 0.0, 0.0, 0.0]` | 位置PIDのkp。各値は有限。このノードは位置指令を送らない |
| `pos_ki` | double[] | `[0.0, 0.0, 0.0, 0.0]` | 位置PIDのki。各値は有限 |
| `pos_kd` | double[] | `[0.0, 0.0, 0.0, 0.0]` | 位置PIDのkd。各値は有限 |
| `reset_encoder_origin` | bool[] | `[false, false, false, false]` | 初期化時にencoder原点とPID状態をリセットするか |
| `reset_encoder_origin_with_switch` | bool[] | `[false, false, false, false]` | スイッチ入力で原点を合わせるか |
| `encoder_origin_switch_state` | bool[] | `[true, true, true, true]` | 原点合わせのスイッチ一致level |

配列のCLI指定例：

```bash
ros2 run param_checker param_checker --ros-args \
  --params-file /tmp/param_checker_measurement.yaml \
  -p 'encoder_ppr:=[2048, 2048, 2048, 2048]' \
  -p 'encoder_polarities:=[-1, 1, -1, 1]'
```

`pid_state` は `pid` から導出するため独立したオプションではありません。
frame ID、230400 baud、10ms timerは固定です。
このノードには `command_mode`、`wheel_polarities`、`open_loop_output_limit`、
`open_loop_slew_rate_per_sec`、`feedback_timeout_ms`、`auto_restart` のオプションはありません。

### 指定・確認に使うROS 2 CLIオプション

| 指定 | 用途 |
|---|---|
| `--ros-args` | ros2 runでROS用引数の指定を開始 |
| `-p 名前:=値` | ROSパラメータを指定。複数回指定可能 |
| `--params-file ファイル` | ROSパラメータYAMLを読む |
| `-r 元topic:=新topic` | topicをremap。例：`-r controller/joy:=/joy` |
| `--log-level info` | ログレベル。最終gain表を見る場合はinfo以上を表示する設定で使用 |
| `ros2 launch ... --show-args` | launch引数と既定値を表示。測定ノードは起動しない |

確認用のコマンドは次のとおりです。

```bash
ros2 launch param_checker param_checker.launch.xml --show-args
ros2 param list /param_check
ros2 param get /param_check pid
ros2 param dump /param_check
```

`ros2 param` の確認コマンドは測定ノードが動いている間に実行します。
同じROS domainの別端末を使用してください。終了後にはノードのパラメータを取得できません。

## 12. 位置制御のゲイン測定と動作確認

[参照リポジトリの二段階VRFT](https://github.com/TNCT-Mechatech/stm32_board_pid_pos_vel_control_set/blob/78edb8b795f9315d4492b7b9716ea78b12200a4f/src/param_checker/README.md)
に合わせ、内側の速度PIを先に用意し、外側の位置Pを測定します。
STMの位置制御は `目標生カウント − 現在生カウント` にKpを掛けてrpsへ変換し、
速度PIで出力を決めます。位置Ki・Kdと速度Kdは0固定です。

### 速度 → 位置測定 → 位置動作確認

1. 1〜5節の手順で速度用を実行し、同じ実機設定で速度ゲインYAMLを保存します。
   既存の有効な速度ゲインも利用できます。対象wheelのKpは正、Kiは非負である必要があります。
2. コントローラを中立にして、位置用を `position_operation:=tune` で起動します。
   対象モータは `target_motor_index:=0`〜`3` で1台ずつ指定します。
3. `Position tune started:` から既定30秒間、`axes[1]` を小さい正負の入力へ何度か変化させます。
   他のモータにはゼロdutyを送り、メカナム合成は行いません。
4. 最後の位置ゲイン表で「保存済み」を確認します。既定では有効な計測窓20以上、
   Kp>0、学習・検証それぞれの適合度R²≥0.7を満たす場合だけYAMLを更新します。
5. 同じファイルを使って `position_operation:=verify` で起動し、位置制御の動作を確認します。

```bash
measurement_dir="$HOME/.ros/natto/measurements/run01"

# 前段: 速度ゲイン測定。設定YAMLは前節で用意したものを使用
ros2 launch param_checker param_checker_velocity.launch.xml \
  serial_config:=/tmp/param_checker_measurement.yaml \
  gain_output_path:="$measurement_dir/pid_gain.yaml"

# 後段: wheel0の位置Pを同定
ros2 launch param_checker param_checker_position.launch.xml \
  serial_config:=/tmp/param_checker_measurement.yaml \
  gain_output_path:="$measurement_dir/pid_gain.yaml" \
  target_motor_index:=0 \
  position_operation:=tune \
  position_measurement_max_rps:=0.03 \
  position_response_time_seconds:=2.0

# 保存した位置ゲインで、開始位置から±1000カウントの範囲を確認
ros2 launch param_checker param_checker_position.launch.xml \
  serial_config:=/tmp/param_checker_measurement.yaml \
  gain_output_path:="$measurement_dir/pid_gain.yaml" \
  target_motor_index:=0 \
  position_operation:=verify \
  position_range_counts:=1000
```

各コマンドは前のプロセスが終了してから実行します。別ポートの場合は各コマンドへ
`serial_path:=/dev/serial/by-id/実機のポート` を追加してください。
全輪を調整する場合は位置用をwheel0〜3について順番に実行します。

| 位置用の操作モード | Joyから送る値 | STMで使う制御 | 結果 |
|---|---|---|---|
| `tune` | 軸入力×速度上限のrps（ID4〜7のうち対象1台） | 保存済み速度PI | 位置Pゲインを算出・保存 |
| `verify` | 開始生カウント＋軸入力×位置範囲（ID8〜11のうち対象1台） | 保存済み位置P＋速度PI | 実際の位置制御を確認。ゲインファイルを変更しない |

位置ゲインの同定中は保存済み速度PIへ速度指令を送ります。
生カウントを既定100msの重ならない区間で差分し、その区間の平均速度から、
速度PIの応答モデル `G × exp(-L s) / (T s + 1)` を推定します。
指令は各区間内で保持された値として積分し、推定した遅延で指令変更時刻をずらします。
整数カウントを10msごとに微分した速度と、瞬間の速度指令を直接一致させる方法から変更しました。
生カウントは差を取ってからdoubleへ変換し、大きな累積値でも小さな変化を保持します。

前半60%の計測窓で定常ゲインG・応答時定数T・実効遅延Lを推定し、後半40%を
パラメータ推定に使わず検証します。保存判定のR²は学習R²と検証R²の小さい方です。
前半・後半とも正負の指令を複数回含めてください。一定方向だけの入力、停止したままの測定、
途中から機構の特性が変わった測定を、適合度の高い前半だけで採用しないための判定です。

位置Pは、速度応答を含む位置ループの遅い極が目標時定数τに対応するよう決めます。
CPRを`encoder_ppr × 4 × gear_rate`として、
`Kp = (τ − T) / (τ² × CPR × G) × exp(−L/τ)`です。
目標時定数τは推定した`T + L`の3倍以上を要求し、内側の速度応答と分離します。
この条件は目標応答の設定確認であり、実機の摩擦・速度飽和・可動範囲を保証するものではありません。
推定Gが0.5〜1.5から外れる場合、推定Tが探索上限2秒、またはLが上限0.3秒に達する場合も保存しません。

従来の仮想誤差`τ × Δcount / Δt`によるR²は、比較用の「旧VRFT R²」として残します。
新しい保存判定は速度応答モデルの予測精度なので、旧R²とは定義が異なります。

`verify` では、最初のfeedbackの生カウントを開始位置として保持します。
軸0で開始位置、軸+1で開始位置＋範囲、軸-1で開始位置−範囲へ移動します。
**中立に戻すと開始位置へ戻る指令になります。** これは軸からの相対位置指定で、
軸を倒し続けて目標位置を加算する操作ではありません。
範囲の単位はradや回転数ではなく生カウントです。車輪1回転のカウント数は
`encoder_ppr × 4 × gear_rate` です。PPR=2048・ギア比245/6なら約334507カウント/回転です。

### 起動・停止・保存の条件

- 初期化時は対象を速度ゼロ、他の3台をdutyゼロにし、feedback前に絶対位置ゼロへ移動する指令は送りません。
- 対象モータの最初のfeedbackから計時します。開始ボタンはありません。
  起動直後や測定中に対象のfeedbackが `feedback_timeout_ms` 途絶えると停止して異常終了します。
  他のモータやスイッチだけのfeedbackではこの期限を延長しません。
- 指令を10msごとに送信します。Joyが `timeout_ms` 途絶えると、tuneは速度ゼロ、
  verifyはdutyゼロへ切り替えて位置保持を解除します。Joyが復帰すると通常指令へ戻ります。
- 正常終了時も停止します。位置モードからの切替で異なる種類の指令が同じSTM周期に入る場合を考慮し、
  最後のゼロ指令を50ms継続してから終了します。
- 速度ゲインがない、単位が異なる、ゲインが不正な場合はシリアルを開く前に異常終了します。
  参照リポジトリの `wheelN.velocity` 形式も読み込めます。
- `position_measurement_max_rps`、設定YAMLの対象 `max_motor_rps`、ゲイン内の `max_rps` の最小値を使います。
  tuneでは目標速度上限、verifyではSTM位置制御の速度上限になります。
- nested形式に速度応答時間がある場合、位置応答時間はそれより長い値を要求します。
  3倍未満の場合は警告します。
- 測定品質が不十分な場合はCSVと判定を出力し、既存ゲインYAMLを保持します。
  通信断・途中終了時には今回のゲインを保存しません。

### 保存形式と結果の確認

既存の速度ゲインを保持したまま、対象wheelへ `position` を追記します。
位置ゲインはradやrps誤差へ掛ける値ではなく、**生カウント誤差をrpsへ変換する係数**です。
例（数値は説明用）：

```yaml
wheel0:
  kp: 0.12                  # 既存の速度ゲインを保持
  ki: 0.34
  kd: 0.0
  position:
    controller: P
    kp: 0.0000025
    ki: 0.0
    kd: 0.0
    input_unit: encoder_count
    output_unit: rps
    response_time_seconds: 2.0
    max_rps: 0.03
    fit_r_squared: 0.95      # 学習と検証の小さい方
    identification_method: velocity_response_model_v1
    training_fit_r_squared: 0.98
    validation_fit_r_squared: 0.95
    velocity_response_time_seconds: 0.3
    velocity_delay_seconds: 0.04
    velocity_steady_gain: 1.0
    filter_window_seconds: 0.1
    encoder_ppr: 2048
    gear_rate: 40.833332061767578
    encoder_polarity: -1
```

gear_rateはSTMへ送信したfloat値を記録します。保存時は一時ファイルから置換し、
書き込み失敗時に元のYAMLを壊さないようにします。他wheelや速度ゲインは保持します。
別の `gain_input_path` と既存の出力先を指定した場合は、対象wheelの速度設定も読み込み元から
コピーし、同定に使った速度PIと位置Pを一緒に保存します。他wheelは出力先の設定を保持します。
既存の速度用を再実行すると速度ゲインファイルを再生成するため、位置ゲインも再測定してください。
verifyでは保存したPPR・ギア比・極性が現在の設定と一致することを確認します。

| 出力 | 内容 |
|---|---|
| `position_pid_gainN` | 保存に成功した位置ゲイン (`debug_msgs/msg/PidGain`) |
| `encoder_raw_countN` | 対象の64bit生カウント (`std_msgs/msg/Int64`) |
| `vrft_position_wheelN.csv` | tuneの生データ・旧VRFT診断と、平均入力・実測/モデル平均速度・残差・学習/検証区分 |

CSVの既定保存先はゲインYAMLの親ディレクトリです。同じwheelの再測定では上書きします。
verifyはこの測定CSVを作成しません。速度用のシリアル診断CSV設定は位置用にはありません。
`used_for_regression`は旧VRFT診断のフラグです。新モデルに使った窓の終端行には
`model_partition`が`training`または`validation`で入り、それ以外の行はモデル列が空です。

### 位置用launch引数（全10項目）

| launch引数 | 既定値 | 用途 |
|---|---|---|
| `serial_path` | 速度用と同じSTMポート | 接続先 |
| `serial_config` | `config/serial_protocol.yaml` | 共通エンコーダ設定と追加ROSパラメータ |
| `gain_output_path` | 空文字 | 既定は `~/.ros/natto/pid_gain.yaml`。位置ゲインの追記先 |
| `gain_input_path` | 空文字 | 空なら出力先と同じ。別の速度ゲインを読み込む場合に指定 |
| `position_operation` | `tune` | `tune` または `verify` |
| `target_motor_index` | `0` | モータ番号0〜3 |
| `measurement_seconds` | `30.0` | 最初のfeedbackからの実行時間 |
| `position_measurement_max_rps` | `0.03` | 目標速度・位置制御速度の上限 |
| `position_response_time_seconds` | `2.0` | 位置応答の遅い極に対応する目標時定数 |
| `position_range_counts` | `1000` | verify時の開始位置からの±範囲 |

これら9つのROSパラメータはlaunch引数が設定YAMLより優先されます。
他のROSパラメータは `serial_config` の `param_check.ros__parameters` へ記載します。

### 位置用ROSパラメータ

| パラメータ | 型 | コード既定値 | 用途・制約 |
|---|---|---|---|
| `serial_path` | string | 速度用と同じSTMポート | 230400 baud固定 |
| `gain_output_path` | string | 空文字 | 位置ゲイン追記先。既定解決規則は速度用と同じ |
| `gain_input_path` | string | 空文字 | 空なら出力先から読み込む。verifyでもこの読み込み先を使用 |
| `vrft_csv_path` | string | ゲインの親ディレクトリ内 `vrft_position_wheelN.csv` | tuneのCSV。空で無効化。ゲインYAMLと異なるパスを指定 |
| `position_operation` | string | `tune` | `tune` / `verify` |
| `target_motor_index` | integer | `0` | 0〜3 |
| `measurement_seconds` | double | `30.0` | 正の有限値（秒） |
| `position_response_time_seconds` | double | `2.0` | 正の有限値（秒）。tuneの応答モデル |
| `position_filter_window_seconds` | double | `0.1` | 重ならないカウント差分区間（秒）。0.02〜0.5 |
| `position_measurement_max_rps` | double | `0.03` | 正の有限値。他の保存・設定上限でも制限 |
| `position_range_counts` | integer | `1000` | 1〜1000000000。verifyの±位置範囲 |
| `axis_y` | integer | `1` | 操作に使うJoy軸番号。0以上 |
| `invert_axis` | bool | `false` | trueで軸の符号を反転。速度用のy符号反転とは別設定 |
| `joy_deadband` | double | `0.05` | 0以上1未満。軸の絶対値がこれ未満なら0 |
| `minimum_fit_r_squared` | double | `0.7` | 0〜1。保存に必要な適合度 |
| `minimum_samples` | integer | `20` | 保存に必要な計測窓数。モデル推定には別途、学習10窓・検証6窓以上が必要 |
| `timeout_ms` | integer | `300` | 正の整数。Joy期限 |
| `feedback_timeout_ms` | integer | `300` | 正の整数。対象モータのfeedback期限 |
| `stop_mode` | bool | `false` | STMの停止モード |

共通10項目は11節のSTM初期化表と同じです。ただし位置用のコード既定 `max_motor_rps` は
`[1.5, 1.5, 1.5, 1.5]` です。対象のpos_kpは読み込んだ値（tuneでは0）、
全輪のpos_ki・pos_kdは0へ上書きし、対象の速度PIはゲインファイルから設定します。
原点リセット配列は初期化時に送信されます。計測中に原点が変わる設定では、
生カウントの連続性が保たれるよう測定環境を確認してください。

launchを使わず、位置用の全設定をCLIで指定する例：

```bash
ros2 run param_checker position_checker --ros-args \
  --params-file /tmp/param_checker_measurement.yaml \
  -p position_operation:=tune \
  -p target_motor_index:=1 \
  -p gain_output_path:="$HOME/.ros/natto/measurements/run01/pid_gain.yaml" \
  -p axis_y:=1 \
  -p invert_axis:=true \
  -p feedback_timeout_ms:=300

# 引数の確認だけを行う（実機を開かない）
ros2 launch param_checker param_checker_position.launch.xml --show-args
```

## 関連ファイル

- [ノード実装](src/main.cpp)
- [launch](launch/param_checker.launch.xml)
- [速度用launch](launch/param_checker_velocity.launch.xml)
- [位置用launch](launch/param_checker_position.launch.xml)
- [位置用ノード](src/position_main.cpp)
- [位置PのVRFT実装](include/param_checker/position_vrft.hpp)
- [初期化設定YAML](config/serial_protocol.yaml)
- [VRFT実装](include/vrft_auto_tuner/vrft_auto_tuner.hpp)
- [通信移行報告・実機確認事項](../../docs/stm_serial_migration.md)


## 13. 雑巾ラックなど機構モーター1台の調整

`param_checker_rack.launch.py`で、注入STMのmotor index 2だけを調整できます。
他の3台は常にDuty 0です。`operation`を変え、速度PI → 位置P → 位置制御確認の順に実行します。
各コマンドは前のプロセスが終了してから起動してください。

### 設定と開始位置

同じポートを使う`inject2026A`と、それを自動再起動するlaunchを停止します。
ラックを原点スイッチを押していない、可動範囲の中央付近へ配置してください。
この調整ツールは原点出しをせず、開始位置を基準に生カウントの移動量を監視します。
スイッチに到達したら停止して測定を中断します。通常運転のスイッチ原点・角度制御は
`inject2026A`が担当します。

[config/inject_rack.yaml](config/inject_rack.yaml)で、PPR・ギア比・極性とスイッチ入力を実機に合わせます。
既定値は注入STM用の現在の初期化設定に合わせています。独自設定はこのファイルをコピーし、
各コマンドへ `serial_config:=/absolute/path/to/config.yaml` を渡してください。
ゲインの単位と符号が変わるため、エンコーダ設定を変更した場合は速度測定からやり直します。

| 設定 | ラック用既定値 | 内容 |
|---|---:|---|
| `target_motor_index` | 2 | 対象モーター。0〜3 |
| `axis_y` / `invert_axis` | 1 / false | 入力軸と符号反転 |
| `measurement_max_duty` | 0.2 | 速度PI測定のDuty上限。Joy入力±1でDuty±0.2 |
| `measurement_max_rps` | 0.5 | 速度測定中の実測速度の停止しきい値、および保存する速度上限 |
| `measurement_range_counts` | 10000 | 速度測定の開始位置からの生カウント移動量上限。0は無効。位置専用設定がない場合は位置測定にも適用 |
| `limit_switch_index` | 2 | 原点スイッチ入力番号。0〜3、-1は監視無効 |
| `limit_switch_active_state` | true | スイッチが押されたときのレベル。active-lowならfalse |
| `feedback_timeout_ms` | 300 | 対象モーター・監視対象スイッチの返信期限 |
| `timeout_ms` | 300 | Joy指令の期限 |
| `require_neutral_on_start` | true | 位置調整でも中立確認後に計時・動作を開始 |
| `position_measurement_max_rps` | 0.06 | 現在の位置P測定・確認の速度上限。gear=1なら21.6°/秒 |
| `position_filter_window_seconds` | 0.1 | 速度応答モデルに使う重ならないカウント差分区間（秒） |
| `position_range_counts` | 100 | 位置確認の範囲。開始位置から±100生カウント |
| `position_measurement_range_counts` | 1000 | 現在の位置測定・確認の停止しきい値。開始位置から±1000生カウント。0は無効 |
| `minimum_samples` / `minimum_fit_r_squared` | 20 / 0.7 | 保存に必要な有効サンプル数と適合度 |

上記のカウント範囲は角度ではありません。
ラック1°のカウントは`encoder_ppr × 4 × gear_rate × rack_transmission_ratio / 360`です。
実機で動かせる範囲に合わせて設定します。速度測定で実測速度上限に達する場合は、
まずDuty上限・Joy入力を小さくしてください。保存速度上限は`max_motor_rps[対象]`との小さい方です。

### スイッチ入力を確認する

スイッチを離しているのに`measurement origin switch reached`となる場合は、
押下判定の入力レベルか入力番号を確認してください。診断モードは全モーターへDuty 0を送り、
ゲインを変更せず、スイッチ入力の変化を表示します。

```bash
ros2 launch param_checker param_checker_rack.launch.py operation:=inspect
```

スイッチを押す・離す操作をすると、`Switch inputs: [0]=... [1]=... [2]=... [3]=...`が出ます。
変化した番号を`limit_switch_index`へ、押した状態の値を`limit_switch_active_state`へ設定します。
例えば入力2が離すとtrue・押すとfalseなら、`limit_switch_index: 2`、
`limit_switch_active_state: false`です。通常運転の`inject2026A/config/serial_ports.yaml`も
`rack_switch_index: 2`、`rack_switch_active_state: false`に合わせます。
Ctrl+Cで診断を終了してから測定します。設定した入力が変化しない場合は接続先・配線を確認します。

同じ診断モードでは、各モーターの折り返さない生カウントをInt64で配信します。
別ターミナルで対象の値を確認できます（Joyは不要です）。

```bash
ros2 topic echo /encoder_raw_count2
```

全モーターへDuty 0を送り続けるため、このモードのJoy操作では動きません。
機構を手で動かせる場合は、例えばラックを90°動かした前後の値を記録します。
手動で動かせない場合は、速度測定中にも同じトピックを確認できます。
生カウント差をΔcount、ラックの角度変化をΔdegとすると、実測の
`counts_per_rack_revolution = abs(Δcount) × 360 / abs(Δdeg)`です。
`encoder_ppr × 4 × gear_rate × rack_transmission_ratio`と整合するか確認します。
`encoder_count`は1回転で折り返すため、この校正には使いません。
起動時の`Encoder calibration`ログはROSが送る設定であり、STMの受信確認ではありません。
実物の動きに比べて生カウントがほとんど変化しない場合は、対象入力番号、
エンコーダのA/B相配線、機械的な連動を確認します。換算の確認前にPIDの保存基準を緩めないでください。

### 速度PI → 位置P → 確認

```bash
source install/setup.bash

# 1. 速度PIの測定。中立にして開始し、左スティック上下を小さな正負入力へ変化させる。
ros2 launch param_checker param_checker_rack.launch.py operation:=velocity

# 2. 速度ゲインの「Velocity PI saved」を確認後、位置Pを測定。
ros2 launch param_checker param_checker_rack.launch.py operation:=tune

# 3. 位置ゲインの「保存済み」を確認後、位置制御の動作確認。
ros2 launch param_checker param_checker_rack.launch.py operation:=verify
```

ラック用launchの既定は各15秒です。速度測定は対象の返信と中立Joyの確認後に計時を始めます。
ラック用の位置測定・確認も中立後に開始します。Joy途絶後は停止し、再び中立に戻すまで
動作を再開しません。途中の中立・Joy途絶では開始済みの計時は続きます。

位置P測定は保存済み速度PIで速度指令を送り、外側の位置Pを同定します。
`verify`は開始位置＋Joy入力×`position_range_counts`を位置目標として送り、
中立では開始位置へ戻ります。`inject2026A`のボタン解放時の保持操作とは異なります。

位置P測定中はまだ位置Pを使っていません。Joyの解放時は位置保持へ切り替えず、
速度PIへ0 rpsを送るため、速度ループの応答に応じた減速距離が生じます。
位置側の速度を速度PI測定と同じ0.5 rpsにすると、gear=1では180°/秒となり、
約20°の可動範囲に対して大きい指令になります。低速測定例は0.02 rpsです。
8192 counts/revなら±100カウントは±4.39°、±200カウントは±8.79°です。
現在の設定は速度上限0.06 rps・移動限界±1000カウントです。±1000は±43.95°なので、
実際の可動範囲に合わせて制限を設定し、範囲の中央付近から開始してください。

位置側の速度・範囲をlaunch引数で上書きできます。指定しない項目はYAMLの値を使います。

```bash
ros2 launch param_checker param_checker_rack.launch.py operation:=tune \
  position_max_rps:=0.02 position_travel_limit_counts:=200

ros2 launch param_checker param_checker_rack.launch.py operation:=verify \
  position_max_rps:=0.02 position_range_counts:=100 position_travel_limit_counts:=200
```

さらに低速にする場合は`position_max_rps:=0.01`を指定します。
これらの位置用引数は`velocity`と`inspect`には適用しません。
位置側の実効速度上限は、指定値・STM初期化値・保存済み速度ゲインの上限の最小値です。
`verify`は保存済み位置ゲインの速度上限も適用します。実効値を起動ログに表示します。
`position_range_counts`は有効な位置用移動限界より小さく設定します。
`position_response_time_seconds`で目標応答時定数も上書きできます。既定2秒で、
値を大きくするとゆっくり追従する位置Pを同定します。速度の上限とは別の設定です。
`position_filter_window_seconds`でカウント差分の区間幅も指定できます。
既定0.1秒、設定範囲0.02〜0.5秒です。区間を長くするとカウント量子化の影響が小さくなりますが、
同じ測定時間から得られる有効区間数は減ります。

保存先は`~/.ros/natto/inject_pid_gain.yaml`、`ROS_HOME`設定時は
`$ROS_HOME/natto/inject_pid_gain.yaml`です。`gain_output_path:=/absolute/path.yaml`で変更できます。
`serial_path`、`joy_topic`（既定`/inject/joy`）、`target_motor_index`、
`measurement_seconds`もlaunch引数で変更できます。

速度測定ではKdを0固定のPIとして直接同定します。有限でKp>0、Ki≥0、
有効サンプル数・適合度が基準を満たす場合だけ、対象の`wheel2.velocity`とエンコーダ校正を
一時ファイル経由で保存します。他のモーターやメタデータは保持します。
**対象の古い位置ゲインは削除されるため、速度測定後は位置Pを再測定してください。**
位置測定はこのファイルへ`wheel2.position`を追記します。失敗・返信途絶・スイッチ検出・
移動限界で中断した測定は、既存のゲインファイルを変更しません。

測定が完了したら`inject2026A`を再起動します。既定で同じゲインファイルの`wheel2`を読み込みます。
通常運転の`rack_max_angle_deg`などの実機設定は別途`inject2026A/config/serial_ports.yaml`へ設定します。

### 足回りモードと検証

既存の足回りlaunchは既定の`target_motor_index=-1`で、従来のメカナム合成・4台PID測定を維持します。
汎用の速度launchでも`target_motor_index:=0〜3`を渡すと単体PI測定へ切り替わります。
その場合は`pid=false`と機構に合った`serial_config`・`serial_path`・`gain_output_path`を指定してください。
単体PIモードでは測定CSVだけを出力し、指令列は`target_duty`です。
4輪用のシリアル診断CSVは出力しません。

```bash
colcon build --packages-select param_checker --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select param_checker --ctest-args -R 'test_single_motor|test_position_vrft|single_motor_pipeline'
colcon test-result --test-result-base build/param_checker --verbose
```

テストは疑似STMで速度PI保存・位置P保存・位置指令の確認まで実行し、対象外モーターの停止、
Joy途絶と中立復帰、返信途絶、スイッチ、移動限界、同定失敗時のファイル保持を確認します。
実機のモーターは動かしません。
