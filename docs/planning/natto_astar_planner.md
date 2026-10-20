# natto_astar_planner
A*アルゴリズムに基づく経路計画ノードです。
x,y,yawの計画を行います

## 機能
- 2Dグリッドマップ上でのA*経路計画
- 角度を考慮した経路計画
- 経路のスムージング
- 凹凸のフットプリントを考慮した計画
- 経路からの逸脱を検知した自動再計画
- ゴール到達時の経路クリア

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| theta_resolution_deg | int | 15 | 角度の分解能（度） |
| xy_inflation | double | 0.5 | XY方向のインフレーション距離（m）この範囲は絶対に入れない |
| xy_offset | double | 0.1 | XY方向のオフセット距離（m）xy_inflationに追加される範囲、できるだけ入りたくない |
| start_area_bounds | double array | 未設定 | 壁の膨張幅を緩和する矩形 `[min_x, max_x, min_y, max_y]`（map座標、m）。未設定なら緩和なし |
| start_area_xy_inflation | double | xy_inflationと同じ | スタート領域内の地図障害物の膨張幅（m）。0以上、xy_inflation以下 |
| keepout_grid_reflection_x | double array | [] | 未設定なら従来の格子化。`[鏡映軸x]` を指定すると、禁止領域を反転座標で格子化して戻す。赤側だけ `[2.7]` を設定 |
| angular_grid_reflect_yaw | bool | false | yawを `180度−yaw` の角度系で量子化・探索して戻す。赤側だけtrueを設定し、青側の角度ビンとの半ビンの差を防ぐ |
| yaw_offset | double | 0.1 | ヨー方向のオフセット距離（rad） |
| grad_alpha | double | 1.0 | 勾配法のアルファパラメーター。増やすと障害物から離れていく |
| grad_beta | double | 8.0 | 勾配法のベータパラメーター。増やすとなめらかな経路になる |
| grad_gamma | double | 0.0 | 勾配法のガンマパラメーター。増やすともとの経路に従いやすくなる |
| grad_step_size | double | 0.1 | 勾配法のステップサイズ |
| replan_distance_threshold | double | 0.2 | 経路再計画の距離しきい値（m）。ロボットが経路からこの距離以上離れると自動的に再計画を実行 |
| restricted_zone_names | string array | [] | LiDAR無視・進入禁止ゾーン名。詳細は[共通restricted zones仕様](../common/restricted_zones.md)を参照 |

## Restricted zones
`keepout=true` のゾーンは生地図のコピーへ占有値100で追加され、通常障害物と同じinflation、A*、平滑化、footprint衝突判定、free-space探索に使用されます。設定形式と座標規約は[共通restricted zones仕様](../common/restricted_zones.md)を参照してください。

赤側の `restricted_zones_red.yaml` では `keepout_grid_reflection_x: [2.7]` を設定しています。0.05mをfloatで保存した際の誤差により、補給エリアの境界が左右で1セルずれることを補正します。禁止領域の形状をx=2.7mで反転し、青と同じ格子の境界判定で占有マスクを作って赤へ戻します。壁を含む元地図には占有セルを追加するだけで、壁のセルは消しません。障害物判定と禁止領域の膨張判定の両方で同じマスクを使います。青側は未設定のため従来の処理を維持します。

この設定を有効にした赤側では、開始・目標位置やコスト勾配のx座標をセルへ変換する際にも同じ反転格子を使います。初回のx=3.9mのように格子境界に乗る位置が、青の対応位置と1セルずれて姿勢経路に差が出ることを防ぎます。

赤側では `angular_grid_reflect_yaw: true` も設定しています。開始・目標のyawを反転した角度系で量子化し、衝突確認と経路出力時に元の角度系へ戻します。従来の `round(yaw/分解能)` と `(index+0.5)×分解能` の半ビン差が同じ方向へ加算される問題を、青側の計算を変更せずに補正します。footprintの衝突確認は実際の赤側のyawで行います。

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| path | nav_msgs/msg/Path | 計画された経路 |
| costmap | nav_msgs/msg/OccupancyGrid | 使用されたコストマップ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| occupancy_grid | nav_msgs/msg/OccupancyGrid | グリッドマップ |
| goal_pose | geometry_msgs/msg/PoseStamped | 目標位置 |
| current_pose | geometry_msgs/msg/PoseStamped | 現在位置 |
| footprint | geometry_msgs/msg/PolygonStamped | ロボットのフットプリント |
| goal_reached | std_msgs/msg/Bool | ゴール到達フラグ（trueを受信すると経路をクリア） |

## 動作詳細

### 初期位置付近の膨張幅緩和

指定した矩形内のグリッド中心に限り、生地図の壁・障害物からの膨張幅を `start_area_xy_inflation` に変更します。矩形外は通常の `xy_inflation` を使い、`xy_offset` はどちらも同じ値です。領域は固定で、初回の経路だけでなく再計画や帰還時にも適用します。実際の壁とフットプリントの衝突判定は維持し、`keepout=true` の進入禁止ゾーンは領域内でも通常の膨張幅を維持します。

赤・青の自動走行では、既存の `config/map/restricted_zones_red.yaml` / `restricted_zones_blue.yaml` の `/planning/astar_planner` セクションに設定しています。赤は x=3.4〜4.4、青は x=1.0〜2.0、yは両方9.5〜10.5で、膨張幅を通常の0.40 mから0.30 mに緩和します。共通マップの設定は変更しません。

### 経路再計画機能
10Hzのタイマーで定期的にロボットの現在位置と計画済み経路の最短距離を計算します。この距離が`replan_distance_threshold`パラメーターで設定された値を超えると、自動的に経路を再計画します。

また、同じゴールを再度受信した場合でも、ロボットが経路を追従できている（しきい値以内の距離）場合は、再計画を行わずに現在の経路を維持します。これにより、不要な再計画を防ぎ、スムーズな動作を実現します。

### ゴール到達時の処理
`goal_reached`トピックに`true`が送信されると、計画済みの経路をクリアします。これにより、ゴール到達後の不要な経路表示を防ぎます。
