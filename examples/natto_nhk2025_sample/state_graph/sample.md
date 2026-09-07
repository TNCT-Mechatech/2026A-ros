```mermaid
stateDiagram-v2

[*] --> 3秒待つ : wait(duration_sec = 3.0)
3秒待つ --> 旗ねらい : set_pose(x = 4.7, y = 2.0, yaw = 90.0)
旗ねらい --> 5秒待つ : wait(duration_sec = 5.0)
5秒待つ --> 左下隅 : set_pose(x = 0.8, y = 0.8, yaw = 0.0)
左下隅 --> 左上隅 : set_pose(x = 0.8, y = 8.5, yaw = 0.0)
左上隅 --> 帰還 : set_pose(x = 3.9, y = 10.0, yaw = 0.0)
帰還 --> [*]

```
