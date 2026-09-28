# ボタンで移動先を選ぶ自動走行

`mecanum_auto.launch.xml`、`mecanum_auto_blue.launch.xml`、`mecanum_auto_red.launch.xml` は、既定で `navigation_mode:=buttons` を使用します。フィールド固定の目標位置・向きを選び、現在位置から A* で経路を作って移動します。

## 操作

| ボタン | 青側 | 赤側 | 共通マップ |
| --- | --- | --- | --- |
| 0 | 旗ねらい | 旗ねらい | 旗ねらい |
| 1 | 帰還 | 帰還 | 帰還 |
| 2 | 旗後ろ | 未設定・無効 | 左下隅 |
| 3 | 遠方 | 未設定・無効 | 左上隅 |
| 16（長押し） | 自動走行の許可 | 同左 | 同左 |

ボタン番号は `/controller/joy` の `buttons` 配列の添字です。0・1の電源ON/OFF割当は削除しました。電源機能の割当がなければ `button_manager` は電源指令を送信しません。

1. ボタン16を押しながら、移動先ボタンを一度押します。
2. 16を押している間、自動で移動します。到着したら停止して待機します。
3. 移動中に別の移動先ボタンを押すと、現在の要求を取り消して新しい経路を作ります。
4. 16を離すと移動を取り消し、手動スティック操作へ戻ります。スティックを倒していれば手動で動きます。
5. 再び16を押しても移動は再開しません。移動先ボタンを押し直してください。

移動中の同じ地点の再選択、長押しによる再送、許可していない間の入力予約は行いません。移動先ボタンを複数同時に押した場合は無視します。起動・再接続時に押されていた移動先ボタンは、離してから押し直す必要があります。未定義の地点への入力は無視し、現在の移動は継続します。

## 起動

```bash
source install/setup.bash
ros2 launch natto_nhk2025_sample mecanum_auto_blue.launch.xml
```

赤側は `mecanum_auto_red.launch.xml`、共通マップは `mecanum_auto.launch.xml` を使用します。既存のシリアル・LiDAR等の起動引数は利用できます。

Mermaid による従来の順次走行に戻す場合：

```bash
ros2 launch natto_nhk2025_sample mecanum_auto_blue.launch.xml navigation_mode:=sequence
```

ボタン方式では動的経路計画を有効にし、静的CSV経路追従は無効にする必要があります。Mermaid の実行ノードは起動しません。汎用の `natto.launch.xml` は既定値を `sequence` としており、他の起動構成は従来どおりです。

## 設定

`examples/natto_nhk2025_sample/config/` に配置しています。

- `destinations_blue.yaml` / `destinations_red.yaml` / `destinations.yaml`: 地点名と座標。
- `goal_buttons.yaml`: 青・赤のボタン割当。
- `goal_buttons_common.yaml`: 共通マップのボタン割当。
- `controller.yaml`: 手動入力と既存の16の長押し設定。0・1の電源割当はありません。

地点ファイルは専用の YAML です。ROS パラメータ YAML の `ros__parameters` は付けません。

```yaml
frame_id: map
destinations:
  flag:
    label: "旗ねらい"
    x: 3.7
    y: 4.7
    yaw_deg: 90.0
```

座標は m、向きは度です。各フィールドの `map` 座標を直接記述します。ボタン方式は `reverse_y:=false` を要求し、地点を二重に反転しません。地点の `label` は説明用で、通信には地点名（例：`flag`）を使います。

```yaml
bindings:
  button_0: flag
  button_1: home
  button_2: behind
  button_3: far
```

`destinations_path` と `goal_buttons_path` 起動引数でファイルを差し替えられます。割当先の地点がそのフィールドにない場合、そのボタンは無効になります。赤側の未設定地点は、実測座標を追加してから使用します。設定変更は再起動時に反映されます。

自動速度・到着許容誤差は既存の `control.yaml` を使います。手動用の `controller_max_xy_speed_m_s` は自動速度の上限ではありません。到着の既存設定は位置2 cm・角度5度および速度指令の収束条件です。実際の車輪速度を測定する到着判定ではありません。

## 構成とキャンセル

- `joy_goal_selector`: `/controller/joy` から、16の許可状態と移動先の押下イベントをまとめた `/navigation/input` を生成。
- `navigation_manager`: 地点を姿勢に変換し、UUID付き `/navigation/goal` を発行。入力・自己位置・追従の監視とキャンセルを担当。
- `astar_planner`: `managed_navigation=true` では要求ID付き `/navigation/path` を発行。無効な目標・経路生成失敗時は目標と経路を消去。
- `holonomic_pure_pursuit`: 対応するIDの `/navigation/feedback` に速度指令と到着結果を返す。
- `navigation_manager`: 現在の要求に一致する速度指令だけを `/control/command_velocity` に出力。
- `twist_selector`: `/controller/allow_auto_drive` に従い、手動・自動の速度指令を選択。

ボタン方式では有効な許可状態の発行元は `navigation_manager` です。既存 `button_manager` の許可トピックは `/controller/legacy_allow_auto_drive` に接続し、監視による許可解除と競合させません。

切り替え時は旧IDを無効にして停止指令を出し、新しい経路の追従応答を待ちます。旧IDの速度指令・到着・失敗結果は無視します。キャンセル・到着・失敗時はプランナーの目標も消去するため、マップ更新によって旧経路が復活しません。ボタン方式では従来の `/goal_pose` や `goal_reached` の Bool は走行要求・完了処理に使用しません。

目標が障害物・進入禁止領域内にある場合は失敗とし、別の空き地点へ目標を自動補正しません。位置が同じでも向きが異なる要求は受け付けます。

監視時間の既定値（ノードパラメータで変更可）：

| ノード | パラメータ | 秒 |
| --- | --- | --- |
| joy_goal_selector | joy_timeout_sec | 0.5 |
| navigation_manager | input_timeout_sec | 0.5 |
| navigation_manager | pose_timeout_sec | 0.5 |
| navigation_manager | plan_timeout_sec | 5.0 |
| navigation_manager | feedback_timeout_sec | 0.5 |
| navigation_manager | travel_timeout_sec | 60.0 |

監視はメッセージ受信時刻と単調時計を使用します。通信断や自己位置停止後、受信が復旧しただけでは移動を再開しません。走行許可中の計画待ち・到着・失敗では自動側にゼロ速度を出力します。

## 状態確認

```bash
ros2 topic echo /navigation/status
```

`state` は `idle`, `planning`, `moving`, `arrived`, `canceled`, `failed`。`destination`, `request_id`, `reason` から対象地点と停止理由を確認できます。最後の状態は遅れて購読したクライアントにも配信します。

## 検証

```bash
colcon build --packages-up-to natto_navigation natto_joy natto_launch natto_nhk2025_sample
source install/setup.bash
ROS_DOMAIN_ID=171 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  NATTO_NAVIGATION_INTEGRATION=1 \
  python3 -m pytest -q src/behavior/natto_navigation/test
```

統合テストは実際のA*・経路追従・指令選択ノードと、ボタン入力・移動管理ノードを接続し、自己位置を模擬します。ハードウェアは起動しません。通常移動、同じ位置での旋回、到着、移動先変更、古い到着通知、16の解除・再許可、障害物への目標設定、キャンセル後のマップ更新、コントローラ切断、自己位置の更新停止を検証します。
