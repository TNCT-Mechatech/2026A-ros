# inject2026A

2026-10-17: [STM設定適用確認 v2](../../docs/stm_config_id.md)へ対応。
Duty運転でも初期化ACKまたは周期通知のID照合が必要です。接続時だけの1回送信は共通の確認・再送処理へ更新しました。

Ros2 Humble

`main` は1プロセス内で2つのROS 2ノードを実行し、注入機構用とロール機構用の
2台のマイコンへ、それぞれ別のUSBシリアルポートで接続します。

- Joy入力: `inject/joy`（両ノード共通）
- 注入機構フィードバック: `inject/fb_speed_topic`
- ロール機構フィードバック: `roll/fb_speed_topic`
- ロール機構の送信Duty: `roll/command_duty`（`std_msgs/msg/Float32MultiArray`）

発射用マイコンへ送ったDutyは次のコマンドで確認できます。

```bash
ros2 topic echo /roll/command_duty
```

`data` はmotor index 0〜3の順です。先頭がaxes[6]側、2番目がaxes[7]側で、
両入力が0.45なら先頭2要素は約`[-0.45, 0.45]`です。値はsigned Duty（-1〜1）で、
0.45は45%を表します。全4モーターのシリアル書き込みが成功した後に10ms周期で公開し、
Joy途絶・中立待ちなどで停止指令を送った場合は0を公開します。
ポート切断中や書き込み失敗時は公開しません。この値はPCから送った指令であり、
STMの受信確認・実際のPWM・実測回転速度ではありません。
system monitorの既定rosbag記録対象にも含まれます。

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

この実行ファイルは注入とロールの両方に接続します。ポートを開けない場合も
プロセスを終了せず、その機構だけ再接続を試みます。接続成功・失敗のログに対象パスを表示します。
USBポートの変更では設定変更は不要ですが、基板を交換した場合は識別番号の更新が必要です。

`roll2026A` の単独ノードは同じロール用ポートを使うため、この実行ファイルと同時には起動しないでください。

`Permission denied` が出る場合は、[シリアルポートの権限設定](../natto_robot_hardware/README.md#初回のシリアルポート権限設定)を行ってください。

## エンコーダ故障時のDuty操作

launch引数`inject_pid`で雑巾ラックの制御を切り替えます。既定は`true`です。

```bash
# エンコーダを使わずに操作
ros2 launch inject2026A inject.launch.xml inject_pid:=false

# 調整済みPIDで位置制御（既定: ボタン4/5の角度指定）
ros2 launch inject2026A inject.launch.xml inject_pid:=true

# 通常の自動起動launchでも同じ引数を指定（赤・青版も対応）
ros2 launch natto_nhk2025_sample mecanum_auto.launch.xml inject_pid:=false
```

`false`では□を押すとDuty +0.2、△を押すとDuty -0.2、解放・同時押しでDuty 0です。
Duty上限は`rack_open_loop_duty`で設定します（0より大きく1以下）。
PIDゲインファイル・エンコーダ返信・原点出し・角度設定を必要とせずに動作します。
角度保持と角度による可動範囲制限は行いません。Joy途絶時には停止し、中立へ戻すまで再開しません。
装填・送り出しとロールは従来のDuty操作を継続します。

`true`では原点スイッチと位置PIDを使います。調整済みPIDゲインが必要です。
切り替えは起動時に行い、変更する場合はlaunchを終了して起動し直します。
`ros2 run`では`inject_pid`パラメータを指定でき、既定は`true`です。

## ボタン4・5で指定角度へ移動

通常launchでは`rack_angle_profile:=true`が既定です。
`buttons[4]`と`buttons[5]`の新しい押下で、別々に設定した角度へ移動します。
角度は原点スイッチを0°とする絶対角度で、ボタンを押すたびに加算する値ではありません。

```bash
source install/setup.bash
ros2 launch inject2026A inject.launch.xml inject_pid:=true \
  rack_target_angle_4_deg:=5.0 rack_target_angle_5_deg:=10.0
```

1. `param_checker`を終了し、コントローラを中立にして起動します。同じシリアルポートを同時に開きません。
2. 原点スイッチ入力2がtrueで50ms連続すると、その時の生カウントを0°にします。
   原点未確認時は△を押している間だけ原点へ寄せます。接触したら停止します。
3. 全ボタンを一度離し、`buttons[4]`なら5°、`buttons[5]`なら10°へ移動します。
   台形加速で軌道を生成し、速度PIと位置補正で移動・保持します。
4. ボタン解放後も移動・保持を続けます。台形軌道の生成中（`moving`）の追加押下と同時押しは無視します。
   到達確認中（`settling`）や保持中（`holding`）は、ボタンを押し直すと実測位置から次の目標へ移動します。
   `motion_timeout`で停止した場合も、ラック操作のボタンをすべて離してから押し直すと再指令できます。
   返信が途絶えて原点を失った場合は、原点の再確認が必要です。

`buttons[2]`（□）／`buttons[3]`（△）の通常操作も併用できます。
原点取得後は、押している間だけそれぞれDuty `+rack_manual_duty`／`-rack_manual_duty`で動きます（既定0.2）。
自動移動・保持中でも手動操作を優先して解除し、解放・同時押しでDuty 0にします。
手動操作を離しても前の自動移動は再開しません。button4／5の次の押下で、実測位置から指定角度へ移動します。
原点未確認時の△は従来の原点出し操作です。原点スイッチへの接触と可動範囲の端ではその方向の手動移動を停止します。
手動DutyにもJoy・返信の途絶監視が適用されます。`rack_manual_duty: 0.0`で原点取得後の手動操作を無効化できます。

目標角度はlaunch引数で設定し、両方とも`0〜rack_max_angle_deg`の範囲内にします。
例えば`rack_target_angle_4_deg:=0.0 rack_target_angle_5_deg:=10.0`なら、原点と10°の往復です。
明示したlaunch引数はYAMLの同名設定より優先され、未指定の角度・方向はYAMLの値を使います。
変更する場合は終了して起動し直します。

| 設定 | 現在のYAML値 | 内容 |
|---|---:|---|
| `rack_angle_profile` | true | ボタン4/5の角度指定。`rack_count_profile`より優先 |
| `rack_target_angle_4_deg` | 20.0 | ボタン4の最終角度（度）。launch引数でも指定可能 |
| `rack_target_angle_5_deg` | 30.0 | ボタン5の最終角度（度）。launch引数でも指定可能 |
| `rack_max_angle_deg` | 30.0 | 原点からの可動範囲上限。実機に合わせる |
| `rack_speed_deg_s` | 15.0 | 目標角速度上限（度/秒） |
| `rack_acceleration_deg_s2` | 30.0 | 加減速の大きさ（度/秒²） |
| `rack_velocity_feedforward` | true | 速度PIの応答モデルで速度・加減速を補償。角度指定モードのみ |
| `rack_manual_duty` | 0.2 | button2／3の手動Duty。自動移動に優先、解放・同時押しで停止。0で無効 |
| `rack_max_rps` | 0.2 | 指令速度上限。保存済みPID上限との小さい方を使う（現在0.06 rps） |
| `rack_position_tolerance_deg` | 0.2 | 到達判定の許容誤差。カウントへ切り捨て換算 |
| `rack_range_tolerance_deg` | 1.0 | 可動範囲外の停止判定の許容誤差。到達精度と独立。カウントへ切り捨て換算 |
| `rack_motion_timeout_s` | 20.0 | 台形目標の生成から実測到達確認までの期限 |
| `rack_angle_count_sign` | 1 | 原点から離れると生カウントが増えるなら1、減るなら-1。launch引数でも指定可能 |
| `rack_transmission_ratio` | 1.0 | エンコーダ設定の出力軸からラックまでの追加減速比。直結なら1 |

速度・加速度・角度範囲などは`config/serial_ports.yaml`の
`inject_serial_node.ros__parameters`へ設定します。
カウント換算には調整済みゲインのPPR・ギア比・エンコーダ極性を読み込みます。
AMT102のPPR2048・4逓倍・ギア比1・直結なら8192カウント/回転です。
現在の符号では5°が約＋114カウント、10°が約＋228カウントです。
整数の位置指令へ丸めるため、1カウント約0.044°の分解能になります。

原点取得後にスイッチへ向かって動く場合は、`rack_angle_count_sign`を反転します。
目標角度は0〜上限の正の値のままにし、PID調整済みの`encoder_polarities`は変更しません。
例えば、現在の+1方向を小さな角度で確認する起動例は以下です。

```bash
ros2 launch inject2026A inject.launch.xml rack_angle_count_sign:=1 \
  rack_target_angle_4_deg:=1.0 rack_target_angle_5_deg:=2.0
```

負の目標角度は範囲外として起動時に拒否します。

台形プロファイルの速度上限がSTMまたは保存済みゲインの速度上限を超える場合、
起動時に拒否します。目標5°・速度5°/秒・加速度10°/秒²なら、目標生成は約1.5秒です。
実際の到達時間は位置P・速度PIの追従に依存し、目標生成後も到達確認まで制御指令を送ります。
現在の速度15°/秒・加速度30°/秒²では、原点から20°の目標生成が約1.83秒、30°が約2.5秒です。
`rack_velocity_feedforward=true`では、保存済み速度PIをSTMで動かし、ROSから速度指令を送ります。
保存済みモデルの時定数T・遅延L・定常ゲインGを使い、指令は
`(参照速度 + T × 参照加速度) / (CPR × G) + Kp × (遅延Lに合わせた参照位置 − 実測位置)`です。
速度と加減速を先に指令し、位置Pは軌道からのずれを補正します。保存済みKp・Kiは変更しません。
指令は実効速度上限に制限し、参照速度だけを追加する方式で生じる減速時の遅れを補償します。

以前の位置目標だけを送る方式では、残り1°に対する速度指令が約0.42°/秒でした。
目標時定数2秒の位置Pでは目標に近づくほど遅くなるため、速度上限だけを上げても
最後の追従は速くなりません。測定済み速度モデルによる試験では、現在の補償で
20°・30°とも約2〜3秒で許容誤差内へ到達します。実機の摩擦・負荷・モデル誤差により変わります。
モデル情報のない古いゲインファイルでは、位置Pを再測定してから補償を有効にします。
`rack_velocity_feedforward: false`で従来のSTM位置Pによる追従へ戻せます。
カウント指定・角度の□/△操作・PID OFFでは、この補償を使用しません。
実測位置が`0〜rack_max_angle_deg`を`rack_range_tolerance_deg`の許容幅を超えて外れると
`travel_limit`で停止します。現在は1.0°を許容し、到達判定は`rack_position_tolerance_deg=0.2`のままです。
目標角度と手動操作の端点は引き続き`0〜rack_max_angle_deg`に制限します。
この制限は返信から判定するため、機構の慣性や原点出しのDutyによる移動は別途考慮します。

```bash
ros2 topic echo /inject/rack/angle_deg
ros2 topic echo /inject/rack/target_angle_deg
ros2 topic echo /inject/rack/goal_angle_deg
ros2 topic echo /inject/rack/state
```

`angle_deg`は実測角度、`target_angle_deg`は台形プロファイル途中の角度目標です。
`goal_angle_deg`は押したボタンの最終目標です。移動・到達確認中は1秒ごとに
実測角度・参照角度・最終目標・位置誤差・速度指令・返信速度・Dutyをログへ出します。
生カウントの診断トピックも継続して配信します。
Joy途絶時は停止し、全ボタンを離して押し直すまで再開しません。
対象モーターまたはスイッチ返信の途絶・再接続後は原点を再確認します。
移動限界・動作期限超過で停止した場合は、原点の再確認または再起動が必要です。

## ボタン4で原点から−30カウントへ移動（カウント指定）

`rack_angle_profile:=false`かつ`rack_count_profile: true`では、注入STMのmotor index 2を制御します。
`param_checker_rack.launch.py operation:=inspect`は確認専用で、移動操作は
`inject2026A`の通常launchで実行します。同じシリアルポートを使うため同時起動しません。

1. `param_checker`で保存したwheel2の速度PI・位置Pゲインを用意します（保存手順は後述）。
2. コントローラを中立にして`ros2 launch inject2026A inject.launch.xml inject_pid:=true rack_angle_profile:=false`で起動します。
3. `inject/switch_feedback`の`data[2]`が`true`で50ms連続すると、その時の生カウントを原点にします。
   原点未確認時は△を押している間だけ`rack_homing_duty`で原点へ寄せられます。
   スイッチ接触時は直ちにDuty 0になります。原点出しのDutyの符号は実機の方向に合わせてください。
4. 一度ボタンを離し、`buttons[4]`を新たに押します。原点に対する相対位置−30カウントへ移動します。
   原点の生カウントが225なら最終位置指令は195です。ボタン解放後も動作を続け、押しっぱなしでは再起動しません。
5. 目標到達後は位置PIDで保持します。台形軌道の生成中は追加押下を無視し、到達確認中は次の目標を受け付けます。
   原点取得後の□/△は、このモードで目標を変更しません。

10ms周期で台形速度の位置目標を生成し、STMへ整数の生カウント位置指令を送ります。
短い移動では最高速度に達しない三角形の速度になります。既定の原点から−30への
移動時間は約3.5秒、到達確認はさらに100msです。台形にするのは連続値の目標速度で、
実測の動きはPID調整と機構に依存します。整数カウントの位置指令は丸められます。
原点はROS側のオフセットとして保存し、動作・保持中はスイッチ状態で上書きしません。

`config/serial_ports.yaml`の`inject_serial_node.ros__parameters`で調整します。

| パラメータ | 既定値 | 内容 |
|---|---:|---|
| `rack_count_profile` | true | `rack_angle_profile=false`でボタン4のカウント移動。有効時は角度校正を使用しない |
| `rack_target_relative_count` | -30 | 原点からの最終生カウント差。−30度ではない |
| `rack_max_speed_counts_s` | 10.0 | 目標速度上限（カウント/秒） |
| `rack_acceleration_counts_s2` | 20.0 | 加速・減速の大きさ（カウント/秒²） |
| `rack_position_tolerance_counts` | 1 | 到達判定の許容誤差（±カウント） |
| `rack_settle_ms` | 100 | 許容誤差内で連続して到達確認する時間 |
| `rack_motion_timeout_s` | 20.0 | 動作開始から到達確認までの期限（コード既定10秒） |

`rack_max_rps`と保存済みゲインの速度上限もSTM側で適用します。
カウント速度がSTM側の速度上限を超える設定は起動時に拒否し、
プロファイルと到達確認が動作期限内に収まらない設定では移動を開始しません。
角度換算は使いませんが、STM速度PIDには保存したPPR・ギア比・極性が適用されます。
以前の観測値「約20度で7カウント」が同じ軸・条件なら、30カウントは約86度に相当します。
実機の移動量に合わせて目標値を設定してください。

```bash
ros2 topic echo /inject/rack/relative_count
ros2 topic echo /inject/rack/target_relative_count
ros2 topic echo /inject/rack/state
```

`relative_count`は原点を0とした実測生カウント差、`target_relative_count`は
途中の位置目標です（両方`std_msgs/Int64`、原点確認済み・返信正常時のみ配信）。
状態は`ready` → `moving` → `settling` → `holding`です。
Joy途絶ではDuty 0にし、ボタンを離して新たに押すまで再開しません。
ラックのmotor 2またはスイッチ返信の途絶・USB再接続では原点を破棄し、再確認が必要です。
動作期限超過は`motion_timeout`で停止します。返信と原点が有効なら、操作ボタンをすべて離し、
新たに押すことで再指令できます。押しっぱなしでの再開はしません。可動範囲や原点出しの異常はこの操作では解除しません。
PIDゲインが無効なら`configuration_missing`となり、ボタン4は動作しません。
`inject_pid:=false`ではボタン4の自動移動を使わず、従来の□/△によるDuty操作になります。

## 雑巾ラックの角度制御（従来モード）

`rack_count_profile: false`をYAMLへ設定し、`rack_angle_profile:=false`で再起動すると、
以下の□/△による角度操作になります。

注入STMの **motor index 2** を位置制御します。原点スイッチはSTMの
**switch input index 2** が既定で、`rack_switch_index`で変更できます。原点を0°、可動範囲を
`0〜rack_max_angle_deg` とします。ロールSTMのmotor 2は従来どおりDuty制御です。

1. コントローラを中立にして起動します。
2. 原点未確認なら **△を押している間だけ** `rack_homing_duty` でスイッチへ寄せます。
   □と同時押し、ボタン解放、スイッチ検出時はDuty 0です。
3. スイッチを既定50ms連続で検出したら、そのときの生カウントを0°として記録します。
   起動時からスイッチが押されていれば、この確認だけで原点が確定します。
4. ボタンを一度離します。以降は□で目標角度を増やし、△で減らします。
   解放・同時押しでは目標角度を保持し、位置指令を10msごとに再送します。

STMのエンコーダ値はリセットせず、ROS側で原点カウントを差し引きます。
ラックに対する `reset_encoder_origin` / `reset_encoder_origin_with_switch` は常に無効にし、
STMが位置制御中の座標を突然変更することを防ぎます。

### 実機で設定する項目

`config/serial_ports.yaml` の `inject_serial_node.ros__parameters` に設定します。
コード既定の`rack_transmission_ratio`と`rack_max_angle_deg`の0は未校正を表し、ラックを動かしません。
現在の設定YAMLでは直結1.0、可動範囲30°を指定しています。
PIDファイルが未調整・不正な場合もラックだけ停止し、他の機構のDuty操作は継続できます。

| パラメータ | 既定値 | 内容 |
|---|---:|---|
| `rack_transmission_ratio` | 0.0 | 設定済み`gear_rate`の出力軸回転数 / ラック回転数。直結なら1.0、さらに2:1で減速なら2.0 |
| `rack_angle_count_sign` | 1 | 角度が増えるとSTMの`encoder_raw_count`が増えるなら1、減るなら-1。STM適用済みの極性とは別 |
| `rack_max_angle_deg` | 0.0 | 原点から動かせる角度上限。実機の可動範囲に合わせる |
| `rack_speed_deg_s` | 15.0 | ボタンを押している間の目標角度変化速度 |
| `rack_max_rps` | 0.03 | STM位置制御の速度上限。ゲインファイルの上限がさらに小さければそちらを採用 |
| `rack_homing_duty` | -0.1 | △で原点へ向かうDuty。実機の回転方向に合わせる |
| `rack_homing_timeout_s` | 10.0 | 原点出し開始からの期限。ボタン解放でも計時は継続 |
| `rack_switch_index` | 2 | 原点スイッチのSTM入力番号（0〜3） |
| `rack_switch_active_state` | true | 原点スイッチが押されたときのSTM入力レベル。active-lowならfalse |
| `rack_switch_debounce_ms` | 50 | スイッチ検出から原点確定までの連続検出時間 |
| `rack_feedback_timeout_ms` | 200 | motor 2とスイッチ、それぞれの返信期限 |
| `rack_pid_gain_path` | 下記 | ラックの調整済みゲインYAML |

角度への換算は次の式です。`gear_rate`がラックまでの全減速比をすでに含む場合、
`rack_transmission_ratio` は1.0とし、減速を二重に計算しないでください。

```text
counts_per_degree = rack_angle_count_sign × encoder_ppr × 4 × gear_rate
                    × rack_transmission_ratio / 360
目標生カウント = 原点生カウント + round(目標角度 × counts_per_degree)
```

`rack_pid_gain_path` の既定保存先は `~/.ros/natto/inject_pid_gain.yaml`、
`ROS_HOME` 設定時は `$ROS_HOME/natto/inject_pid_gain.yaml` です。
`param_checker`で調整した **wheel2の速度PIと位置P** を読み込みます。
旧形式の `wheel2: {kp, ki, kd, position: ...}` と、`schema_version: 2` の
`wheel2.velocity` / `wheel2.position` 形式に対応します。保存済み速度Kdは0で送信します。
位置調整時のPPR・ギア比・極性が記録されていれば、それもSTMに反映します。
ゲインファイル変更後はこのノードを再起動します。

速度PIを先に調整・保存し、そのファイルへ位置Pを追記します。
調整中は `inject2026A` を終了し、[ラック用param_checkerの手順](../param_checker/USAGE.md#13-雑巾ラックなど機構モーター1台の調整)
に従ってエンコーダ・スイッチ設定を合わせてください。注入STMのmotor 2と機構別保存先を
指定済みのlaunchで、次を1つずつ実行します。

```bash
ros2 launch param_checker param_checker_rack.launch.py operation:=velocity
ros2 launch param_checker param_checker_rack.launch.py operation:=tune
ros2 launch param_checker param_checker_rack.launch.py operation:=verify
```

速度PIを再測定すると対象の古い位置Pは削除されます。位置Pの保存完了後に
`inject2026A`を再起動すると、既定で同じ`inject_pid_gain.yaml`を読み込みます。

### 停止と復帰

Joy指令が `timeout_ms` 途絶えると全モーターをDuty 0で停止し、ラックの保持も解除します。
中立操作後は以前の目標へ戻らず、測定した現在角度から再開します。
ラックは `monitor_serial_updates=false` でもmotor 2とスイッチの返信鮮度を監視します。
他のモーターの返信ではラックの期限を延長しません。

ラックの返信途絶・USB再接続では原点を破棄します。返信が復帰したら中立に戻し、
再度△で原点を確認してください。原点出しの時間超過は停止状態を保持し、
ボタン操作だけでは解除しません。スイッチでの原点確認または再接続・再起動が必要です。
STMをリセット・交換した場合も、ノードを再起動して原点を取り直してください。
`rack_max_rps` は位置制御の上限で、Dutyによる原点出しには適用されません。

| topic | 型 | 内容 |
|---|---|---|
| `inject/rack/state` | `std_msgs/String` | 設定待ち・返信待ち・原点待ち・原点出し・保持・停止・異常 |
| `inject/rack/angle_deg` | `std_msgs/Float64` | 原点からの実測角度。原点確認済み・返信正常時に配信 |
| `inject/rack/target_angle_deg` | `std_msgs/Float64` | 目標角度。原点確認済み・返信正常時に配信 |
| `inject/rack/encoder_raw_count` | `std_msgs/Int64` | 正確な64bit生カウント。返信正常時に配信 |

```bash
source install/setup.bash
ros2 launch inject2026A inject.launch.xml
ros2 topic echo /inject/rack/state
```

ユニットテストと2台の疑似シリアルSTMを使った統合テストは、実機を動かさず実行できます。

```bash
colcon build --packages-select inject2026A --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select inject2026A --ctest-args -R 'test_rack_controller|test_rack_count_controller|rack_serial_integration'
colcon test-result --test-result-base build/inject2026A --verbose
```

## シリアル通信の監視と復帰

注入のラック以外とロールは、マイコンからフィードバックが来ない構成にも対応します。
ラックの角度制御には上記の位置・スイッチ返信が必須です。
`monitor_serial_updates=false` では、未受信や `serial.update()` の非ゼロ戻り値を
理由に停止・切断・プロセス再起動しません。受信できたID=16〜19の速度フィードバックは従来どおり配信します。

| パラメータ | 既定値 | 用途 |
|---|---:|---|
| `monitor_serial_updates` | false | `update()` の戻り値による受信監視を有効にするか |
| `serial_update_failure_limit` | 5 | 受信監視有効時、0以外が連続したときの再接続しきい値 |
| `timeout_ms` | 300 | Joy指令の受信期限 |
| `reconnect_interval_ms` | 1000 | 切断・接続失敗後の再試行間隔 |

回数・時間は正の値を指定します。接続直後はゼロ指令を送り、中立のJoy入力で動作を許可します。
その後の新しい非ゼロ指令を送信します。ラック以外は返信を待つ必要はありません。
Joy指令が `timeout_ms` 以上途絶えた場合や入力が不正な場合は停止します。
接続・再接続・指令途絶からの復帰には中立操作が必要で、古い動作指令は引き継ぎません。
待機中・指令途絶中は接続中のポートへ10ms周期で停止指令を再送します。

シリアル送信の一時的な `EAGAIN` / `EWOULDBLOCK` / `EINTR` と部分書き込みは、
1フレームにつき最大10msの期限内で再試行します。送信済みの部分は重複送信しません。
期限超過や実際のread/writeエラーでは停止処理を行い、ポートを閉じて再接続します。
ポートを開く／設定する処理に失敗してもプロセスを終了せず再試行を続けます。
毎回 `serial_path` を開き直すので、USB再接続後にtty名が変わっても、同じ
`/dev/serial/by-id/` に戻れば接続できます。各機構の再接続は独立しています。
返信を監視しないため、書き込みが成功している間のマイコン無応答は検出できません。
USB切断やPC停止時は停止指令が届かないため、マイコン側にも指令受信ウォッチドッグが必要です。

`/diagnostics` の `receive_monitoring`、`watchdog_state`、`command_age_sec`、
`write_success_count`、`write_failure_count`、`reconnect_attempts` で状態を確認できます。
`feedback_age_sec=-1` はフィードバック未受信を示しますが、既定モードでは異常ではありません。

### 返信するファームウェアでの受信監視（任意）

`monitor_serial_updates=true` のときだけ10ms周期で戻り値を監視します。
`0` なら連続異常回数をゼロに戻し、`0` 以外なら1増やします。
`-1`（完全なフレーム未受信）、`-2`（不正パケット）、`-3`（未登録ID）は
異なる値が混在しても連続として数え、設定回数に達すると停止・再接続します。
正常判定はID=1に限定せず、登録済みフレームの `update()==0` を対象とします。
このモードでは `update()==0` の後に中立操作が必要です。

既定の5回は約50msです。`-1` は未受信周期にも出るため、マイコンの送信周期と揺らぎに
合わせて回数を設定してください。送信成功だけでは連続異常回数をリセットしません。
`serial_update_result`、`serial_update_failure_count`、`serial_update_failure_limit`、
`serial_update_age_sec` が診断に追加されます。
以前の `feedback_timeout_ms`、`startup_feedback_timeout_ms`、`feedback_recovery_count` は
注入・ロールでは使用しません。足回り側の監視設定は別です。

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

既定の返信不要モードでは、ポートが開いて入出力エラーがない状態の継続を復旧と判定します。
受信監視を有効にした場合は正常受信も必要です。指令途絶・ニュートラル待ちだけでは再起動しません。
再実行後はニュートラル指令が必要で、以前の動作指令は引き継ぎません。
注入・ロールは同一プロセスなので、どちらかの長時間不良で両方が再実行されます。
足回りは別プロセスです。通信が戻らない間は1秒の判定期間ごとに再試行します（プロセスの終了・起動時間は別途かかります）。


## STM個別フレーム通信への移行

[通信移行報告](../../docs/stm_serial_migration.md)を参照してください。
従来の注入/ロールのボタン・軸マッピングは維持し、PID無効の出力をID12〜15のsigned dutyに送ります。
接続・再接続ごとにID0〜3のInitialization（pid_state=0）→Duty=0を送ります。
共通SerialBridgeを使用し、ID0〜20すべての登録結果を検査します。
ID16〜19の最新速度は従来のfb_speed_topicへ、ID20の4入力は
`inject/switch_feedback` / `roll/switch_feedback` (`std_msgs/UInt8MultiArray`) へ公開します。
受信は1回の10ms callbackでdrainし、そのバッチの正常/異常を既存watchdogに渡します。
末尾の `-1` は正常なdrain終了なので、そのバッチの受信成功を上書きしません。
`monitor_serial_updates=false` の既存send-only運用設定は維持しています。
新STMの返信を監視する運用ではtrueを指定してください。
エンコーダPPR・ギア比の設定値はSTM既定値であり、注入/ロール機構ごとに確認してください。
