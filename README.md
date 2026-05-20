# ~~Natto Library~~
~~このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです~~

# Notto Library
勝手に無茶苦茶いじりまくって富山高専めちゃってく部用に改造したものです

あんまり他の高専に言わないようにしましょう


## [もともとの公式茨城高専ドキュメントはこちら](https://nitic-robot-club.github.io/natto_library/)

## 依存関係
```bash
sudo apt install ros-jazzy-xacro
```

## サンプル実行
NHK高専ロボコン2025のサンプルデータがあります

```bash
ros2 launch nhk2025_sample nhk2025_sample.launch.xml
```

オムニホイールのサンプルもあります

```bash
ros2 launch nhk2025_sample omni_sample.launch.xml
```


勝手にいじりまくったメカナムホイールのサンプルもあります

```bash
ros2 launch nhk2025_sample mecanum_sample.launch.xml
```