# 実装前の調査 (2026-09-27)

既存の未コミット変更（navigation、hardware、launch等）も含めて確認し、保持した。
全package.xml・launch構成を列挙し、関連nodeのpublisher/subscriberとmessage定義を確認した。

| package | node (標準launchの完全名) | topic | type |
|---|---|---|---|
| lakibeam1 (src/Lakibeam_ROS2_Driver) | /richbeam_lidar_node0, /richbeam_lidar_node1 | /sensing/lidar/laserscan/front, /sensing/lidar/laserscan/rear | sensor_msgs/msg/LaserScan |
| natto_mcl | /localization/mcl/mcl | /localization/current_pose | geometry_msgs/msg/PoseStamped |
| natto_astar_planner | /planning/astar_planner | /planning/path | nav_msgs/msg/Path |
| natto_holonomic_pure_pursuit | /control/holonomic_pure_pursuit | /control/command_velocity (sequence), /navigation/feedback (buttons) | geometry_msgs/msg/TwistStamped, natto_msgs/msg/NavigationFeedback |
| natto_navigation | /navigation/navigation_manager | /navigation/status, /navigation/path, /navigation/goal | natto_msgs/msg/NavigationStatus, NavigationPath, NavigationGoal |
| natto_command_selector | /control/twist_selector | /command_velocity | geometry_msgs/msg/TwistStamped |
| natto_joy / joy | /controller/joy, /controller/joy_to_twist, /controller/button_manager | /controller/joy, /controller/command_velocity, /controller/allow_auto_drive | sensor_msgs/msg/Joy, geometry_msgs/msg/TwistStamped, std_msgs/msg/Bool |
| natto_wheel_odometry | /localization/wheel_odometry/wheel_odometry | /localization/wheel_odometry/odometry | nav_msgs/msg/Odometry |
| inject2026A | /inject_serial_node, /roll_serial_node (同一プロセス) | /inject/joy, /inject/fb_speed_topic, /roll/fb_speed_topic | sensor_msgs/msg/Joy, std_msgs/msg/Float32MultiArray |
| roll2026A | /serial_node (単独版) | /inject/joy, /fb_speed_topic | sensor_msgs/msg/Joy, std_msgs/msg/Float32MultiArray |
| natto_robot_hardware | /serial_motor_bridge | /joint_states, /diagnostics | sensor_msgs/msg/JointState, diagnostic_msgs/msg/DiagnosticArray |

根拠: 各src/*.cpp、natto_launch/launch/components/{control,localization,navigation,sensing}.launch.*、examples/natto_nhk2025_sample/launch/mecanum_auto.launch.xml。

- /scan_front, /scan_rear は全体launchでは使わない。実際のremap先を初期値とする。
- A*はgoal受信時とreplan timerでPathをpublish。goal未指定・到着後は周期更新しない。到着で空Pathを送る。
- buttons方式のstatusはtransient_localのイベント通知。status自体の経過時間はtimeout判定に使わず、publisher/nodeとmanagerの50Hz allow_auto_driveを確認する。
- sequence方式はgoal・Path・goal_reachedを観測。起動途中から監視を開始し、過去のvolatile goal/Pathを受信できない場合は、現在の実行状態を完全には復元できない。
- Pure Pursuitのmin_distance, goal_distance, yaw_error, target_speedは既存計算結果。任意debug出力はこれらをコピーする。
- wheel odometryは車輪から推定した実速度。指令値をactual velocityと呼ばない。
- injectのJoy callbackがwriteし、read/update timerは10ms。SerialBridge→CobsSerial→LinuxHardwareSerialが実writeバイト数を返す。末端writeを透過的に観測してpartial writeも検出する。
- LinuxHardwareSerialはopen失敗でexitする。この挙動は変更しない。外部device検査(ERROR)とdiagnostics heartbeat(STALE)で補完する。
- natto_robot_hardwareには既存diagnosticsがあり、内容を変えず集約する。
- この環境にはROS Jazzy、diagnostic_updater、diagnostic_aggregator、rosgraph_monitor 0.2.3が存在。
- rosgraph_monitorは外部実行時依存とし、制御nodeへリンクしない。RMWやDeadline QoSを変更しない。

## 実装前に整理した変更対象

新規: natto_system_monitor/{CMakeLists.txt,package.xml,src,include,config,launch,test,README.md,INVESTIGATION.md}、natto_msgs/msg/PursuitDebug.msg。

既存: inject2026A/roll2026Aのmain.cpp/main.hpp/CMakeLists.txt/package.xmlとSerialBridgeの読取り専用fd accessor、Pure Pursuitのhpp/cpp、natto_launchの全体launch・control component launch・依存、実機exampleの監視引数。MCL/A*アルゴリズム、Serial内容・周期・復旧挙動は変更しない。

参考: [Jazzy rosgraph_monitor](https://docs.ros.org/en/ros2_packages/jazzy/api/rosgraph_monitor/index.html)、[diagnostic_aggregator API](https://docs.ros.org/en/ros2_packages/jazzy/api/diagnostic_aggregator/index.html)。API・parameterはインストール済みJazzyヘッダ/設定とも照合した。
