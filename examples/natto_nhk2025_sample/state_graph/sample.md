```mermaid
stateDiagram-v2

[*] --> 3秒待つ : wait(duration_sec = 3.0)
3秒待つ --> 旗ねらい : set_pose(x = 4.7, y = 2.0, yaw = 90.0)
旗ねらい --> 帰還 : set_pose(x = 5.2, y = 6.0, yaw = 0.0)

```
