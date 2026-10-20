# natto_holonomic_pure_pursuit
natto_holonomic_pure_pursuit パッケージは、ホロノミックロボットのためのピュアパシュート制御ノードを提供します。

## 機能
- 全方位移動ロボットのためのピュアパシュート制御
- 目標経路と現在位置に基づいて指令速度を計算
- 加速度、減速度、最高速度、最低速度の制約を考慮

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frequency | double | 50.0 | 制御ループの周波数（Hz） |
| lookahead_time_s | double | 1.0 | 先読み時間（秒） |
| min_lookahead_distance_m | double | 0.1 | 最小先読み距離（メートル） |
| max_lookahead_distance_m | double | 1.0 | 最大先読み距離（メートル） |
| yaw_speed_p | double | 1.0 | ヨー速度比例ゲイン |
| curvature_lookahead_distance_m | double | 0.3 | 曲率計算用の固定経路長（メートル）。速度に応じた先読み距離とは独立 |
| curvature_deceleration_p | double | 1.0 | 曲率減速度比例ゲイン |
| min_curvature_speed_m_s | double | 0.3 | 曲率に基づく最低速度（メートル毎秒） |
| yaw_deceleration_p | double | 1.0 | ヨー減速度比例ゲイン |
| max_speed_xy_m_s | double | 3.0 | XY方向の最高速度（メートル毎秒） |
| min_speed_xy_m_s | double | 0.1 | XY方向の最低速度（メートル毎秒） |
| max_speed_yaw_deg_s | double | 180.0 | ヨー方向の最高速度（度毎秒） |
| min_speed_yaw_deg_s | double | 18.0 | ヨー方向の最低速度（度毎秒） |
| max_acceleration_xy_m_s2 | double | 10.0 | XY方向の最高加速度（メートル毎秒毎秒） |
| max_acceleration_yaw_deg_s2 | double | 500.0 | ヨー方向の最高加速度（度毎秒毎秒） |
| goal_deceleration_m_s2 | double | 4.0 | 目標減速度（メートル毎秒毎秒） |
| goal_deceleration_distance_p | double | 1.0 | 目標減速度距離比例ゲイン |
| goal_position_tolerance_m | double | 0.03 | 目標位置許容誤差（メートル） |
| goal_yaw_tolerance_deg | double | 10.0 | 目標ヨー許容誤差（度） |
| goal_speed_tolerance_xy_m_s | double | 0.3 | 目標速度許容誤差XY方向（メートル毎秒） |
| goal_speed_tolerance_yaw_deg_s | double | 30.0 | 目標速度許容誤差ヨー方向（度毎秒） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | 計算された指令速度 |
| lookahead | geometry_msgs/msg/PoseStamped | 先読み位置 |
| goal_reached | std_msgs/msg/Bool | 目標到達状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| current_pose | geometry_msgs/msg/PoseStamped | ロボットの現在位置 |
| path | nav_msgs/msg/Path | 目標経路 |

## 出発時の先読みと曲率

先読み点は、ロボット中心の先読み円と経路の線分との交点で補間します。
同じXY座標で向きだけ違う点を含んでも、最寄りの経路点へ近づくことで先読み距離が縮むのを防ぎます。
姿勢は最短角度で補間し、経路末尾では最終姿勢を使うため、その場旋回にも対応します。

曲率はロボット位置を経路に投影した位置から、`curvature_lookahead_distance_m` の経路長で等間隔の3点を取って計算します。
速度依存の先読み距離や旋回点の数で曲率を測る範囲が変わらないようにしています。
終点までの経路が短い場合は、その残りの範囲で計算します。

実機サンプルの `control.yaml` では、最小先読み距離を0.2 m、曲率計算の経路長を0.3 mとしています。
比較時は出発直後の速度低下、最大経路誤差、停止後の位置誤差を確認してください。

### 保存済みログでの比較

2026-10-07の18:47:22、18:50:17、18:52:07、18:53:16の4走行について、flagへの移動開始0.3〜2秒の654時点を比較しました。
各時点の記録済み姿勢・経路・直前の速度指令を同じ入力として、加速度制限前の目標速度を再計算しています。

| 設定・実装 | 最低目標速度 [m/s] | 0.2 m/s未満の割合 |
| - | - | - |
| 修正前 | 0.100 | 8.1% |
| 最小先読み距離のみ0.2 m | 0.262 | 0% |
| 0.2 m＋線分補間＋固定0.3 mの曲率計算 | 0.340 | 0% |

これは変更後の走行軌跡や実機応答を再現する試験ではありません。
まず同じ初期位置から実走行して出発直後の速度、経路誤差、停止位置を確認し、低速化が残る場合に壁から離れる経由点の追加を検討します。
実装の回帰確認として、経路形状の単体テスト10件と、到着・目標変更・キャンセル・その場旋回等の模擬統合テスト5件が通っています。
