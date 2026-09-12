# 2026A-ros

<div align="center">

### ROS 2 Robot Control System for TNCT Mechatech

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?style=flat&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=flat&logo=c%2B%2B&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?style=flat&logo=ubuntu&logoColor=white)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)

</div>

---

## ~~Natto Library~~

~~このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです。~~

# Notto Library

茨城高専ロボット部の **Natto Library** をベースに、

**勝手に無茶苦茶いじりまくって富山高専めちゃってく部用に改造したものです。**

気づいたら結構別物になってきました。

あんまり他の高専に言わないようにしましょう。

> [もともとの公式 茨城高専 Natto Library ドキュメント](https://nitic-robot-club.github.io/natto_library/)

---

## What is this?

NHK高専ロボコン向けのROS 2ロボット制御・自動走行システムです。

ざっくり言うと、

```text
LiDAR
  │
  ▼
Localization
  │
  ▼
Path Planning
  │
  ▼
Path Tracking
  │
  ▼
Chassis Control
  │
  ▼
ロボット「たぶんこっちです」
```

という感じです。

主に以下をまとめています。

- LiDARによるセンシング
- MCLによる自己位置推定
- A*による経路計画
- Pure Pursuitによる経路追従
- Mecanum / Swerve系の足回り制御
- CAN / Serialによる実機通信
- State Machineによる自動動作管理
- RViz2による可視化
- シミュレーション

> [!NOTE]
> 正しく動かなかった場合、まずTFを疑ってください。
>
> それでも直らなければ `source install/setup.bash` を疑ってください。
>
> それでも直らなければ、もう一度 `colcon build` してください。

---

# Features

## Localization

- Monte Carlo Localization
- Wheel Odometry
- LiDARを利用した自己位置推定
- TFによる座標系管理

## Path Planning

- A* Path Planning
- Restricted Zone
- 経路生成

## Control

- Holonomic Pure Pursuit
- Mecanum / Swerve Chassis Control
- 自動走行
- Joint State管理

## Sensing

- LiDAR
- Dual LiDAR
- LaserScan Filtering
- LiDAR Merge
- LaserScan / PointCloud変換
- RANSACによる形状検出

## Behavior

- State Machine
- 自動走行シーケンス管理

## Hardware

- CAN
- Serial
- STM32
- Raspberry Pi
- 実機Odometry

## Visualization / Simulation

- RViz2
- Simple Simulator
- Debug Visualization

---

# System Overview

```text
                         ┌────────────────┐
                         │    Behavior    │
                         │ State Machine  │
                         └───────┬────────┘
                                 │
                                 ▼
┌────────────┐          ┌────────────────┐
│   LiDAR    │─────────▶│  Localization  │
│Front / Rear│          │      MCL       │
└────────────┘          └───────┬────────┘
                                 │
                                 ▼
                        ┌────────────────┐
                        │    Planning    │
                        │       A*       │
                        └───────┬────────┘
                                 │
                                 ▼
                        ┌────────────────┐
                        │    Control     │
                        │  Pure Pursuit  │
                        └───────┬────────┘
                                 │
                                 ▼
                        ┌────────────────┐
                        │    Chassis     │
                        │Mecanum / Swerve│
                        └───────┬────────┘
                                 │
                                 ▼
                             STM32 etc.
```

---

# Environment

基本環境：

- Ubuntu 24.04
- ROS 2 Jazzy
- C++
- CMake
- colcon

---

# Setup

## Clone

```bash
git clone git@github.com:TNCT-Mechatech/2026A-ros.git
cd 2026A-ros
```

---

## Dependencies

まずROS 2 Jazzyが入っていることを確認してください。

Xacro：

```bash
sudo apt update
sudo apt install ros-jazzy-xacro
```

その他のROS依存関係：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
colcon build --symlink-install
```

ビルド後：

```bash
source install/setup.bash
```

毎回打つのが面倒な場合は `.bashrc` に追加してもOKです。

```bash
source ~/2026A-ros/install/setup.bash
```

ビルドが通ったら勝ちです。

---

# Launch System

システム全体は `natto_launch` から、

```text
Behavior
Common
Simulation
Sensing
Localization
Control
Visualization
```

を組み合わせて起動する構成になっています。

メインLaunch：

```text
src/launch/natto_launch/launch/natto.launch.xml
```

内部では各機能を、

```text
components/
├── behavior.launch.xml
├── common.launch.xml
├── sensing.launch.xml
├── localization.launch.xml
├── control.launch.xml
├── simulation.launch.xml
└── visualization.launch.xml
```

のように分割しています。

必要な機能だけ起動することもできます。

---

# Samples

NHK高専ロボコン2026向けのサンプルデータも残っています。

## NHK2025 Sample

```bash
ros2 launch natto_nhk2025_sample nhk2025_sample.launch.xml
```

## Omni Wheel

```bash
ros2 launch natto_nhk2025_sample omni_sample.launch.xml
```

## Mecanum Wheel

```bash
ros2 launch natto_nhk2025_sample mecanum_sample.launch.xml
```

実機向け：

```bash
ros2 launch natto_nhk2025_sample mecanum_real.launch.xml
```

エンコーダ故障時の手動退避（PID無効、モーター指令は正規化値）：

```bash
ros2 launch natto_nhk2025_sample mecanum_real_open_loop.launch.xml
```

既定では最大出力を `0.2` に制限します。必要な場合は
`open_loop_output_limit:=0.5` のように指定し、`0より大きく1以下` の範囲で変更できます。
自動走行、自己位置推定、センシングはこのlaunchでは起動しません。

自動走行：

```bash
ros2 launch natto_nhk2025_sample mecanum_auto.launch.xml
```

---

# Packages

## Behavior

| Package | Description |
|---|---|
| `natto_state_machine` | 状態遷移・自動走行シーケンス管理 |

---

## Common

| Package | Description |
|---|---|
| `natto_canable` | CAN通信 |
| `natto_map` | 地図情報 |
| `natto_joy` | Joystick入力 |

---

## Localization

| Package | Description |
|---|---|
| `natto_mcl` | Monte Carlo Localization |
| `natto_wheel_odometry` | Wheel Odometry |

---

## Planning

| Package | Description |
|---|---|
| `natto_astar_planner` | A*アルゴリズムによる経路計画 |

---

## Control

| Package | Description |
|---|---|
| `natto_holonomic_pure_pursuit` | ホロノミックロボット向けPure Pursuit |
| `natto_chassis_calculator` | 足回りのホイール指令値計算 |

---

## Sensing

| Package | Description |
|---|---|
| `natto_laser_filter` | LaserScanデータのフィルタリング |
| `natto_lidar_converter` | LiDARデータ変換 |
| `natto_lidar_merger` | 複数LiDARデータの統合 |
| `natto_ransac` | RANSACによる形状検出 |
| `natto_lakibeam_bringup` | Lakibeam LiDARの起動 |

---

## Simulation

| Package | Description |
|---|---|
| `natto_simple_simulator` | 簡易ロボットシミュレータ |

---

## Visualization

| Package | Description |
|---|---|
| `natto_visualization_converter` | RViz2表示用データ変換 |

---

# LiDAR

Lakibeamを使用したLiDAR構成に対応しています。

Front / Rearの複数LiDARを使用し、

```text
/scan_front
/scan_rear
      │
      ▼
 LiDAR Merger
      │
      ▼
Merged LaserScan
```

のように統合して利用できます。

LiDAR関連のLaunchは主に、

```text
src/sensing/
src/Lakibeam_ROS2_Driver/
```

以下にあります。

---

# Localization

自己位置推定には主にMCLを使用します。

```text
Map
 │
 ├──────────────┐
 │              │
 ▼              ▼
LiDAR         Odometry
 │              │
 └──────┬───────┘
        ▼
       MCL
        │
        ▼
    Robot Pose
```

推定結果は経路計画・自動走行に使用されます。

---

# Path Planning

経路計画にはA*アルゴリズムを使用しています。

```text
Start
  │
  ▼
A* Planner
  │
  ├── Map
  ├── Obstacles
  └── Restricted Zones
  │
  ▼
Path
```

生成された経路はPure Pursuitなどの制御系へ渡されます。

---

# Control

経路追従にはHolonomic Pure Pursuitを使用します。

足回りについては、

- Mecanum
- Swerve

などの構成に対応できるようになっています。

```text
Target Path
    │
    ▼
Pure Pursuit
    │
    ▼
Velocity Command
    │
    ▼
Chassis Calculator
    │
    ▼
Wheel Command
```

---

# Directory

ざっくりした構成：

```text
2026A-ros/
├── docs/
│
├── examples/
│   └── natto_nhk2025_sample/
│
├── src/
│   ├── behavior/
│   ├── common/
│   ├── control/
│   ├── launch/
│   ├── localization/
│   ├── planning/
│   ├── sensing/
│   ├── simulation/
│   └── visualization/
│
├── Lakibeam_ROS2_Driver/
├── SerialBridge/
│
├── .gitmodules
└── README.md
```

※ 実際の構成は変更される可能性があります。

---

# Documentation

詳しい各Packageの説明は [`docs/`](docs/) を参照してください。

元になったNatto Libraryのドキュメント：

[茨城高専ロボット部 Natto Library](https://nitic-robot-club.github.io/natto_library/)

先人に感謝しましょう。

そして壊したら直しましょう。

---

# Development

変更したら基本的に、

```bash
colcon build --symlink-install
source install/setup.bash
```

を実行してください。

ROS 2で何かおかしい場合は、

```bash
ros2 node list
ros2 topic list
ros2 topic echo <topic>
ros2 run tf2_tools view_frames
```

あたりを確認すると何か分かるかもしれません。

分からないかもしれません。

---

# Known Issues

```text
[WARN] Robot may behave unexpectedly.
[WARN] TF tree may contain emotional damage.
[WARN] LiDAR may detect things you didn't want it to detect.
[INFO] Running colcon build again...
[INFO] It worked for some reason.
```

---

# Troubleshooting

## `Package '<package>' not found`

まず：

```bash
source install/setup.bash
```

それでもダメなら：

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## TFがおかしい

確認：

```bash
ros2 run tf2_tools view_frames
```

そして祈ります。

---

## LiDARが見えない

```bash
ros2 topic list | grep scan
```

```bash
ros2 topic hz /scan
```

デバイスも確認：

```bash
ls /dev/ttyUSB*
```

---

## 何も分からない

```bash
reboot
```

意外と効きます。

---

# Original Project

このリポジトリは、茨城高専ロボット部の **Natto Library** をベースにしています。

- Original Project  
  https://github.com/NITIC-Robot-Club/natto_library

- Documentation  
  https://nitic-robot-club.github.io/natto_library/

多大なる感謝。

---

# License

Apache License 2.0
