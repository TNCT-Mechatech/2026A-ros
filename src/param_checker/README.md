# param_checker

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
