# urdf_seminar

[URDF入門セミナー](https://rt-net.jp/service/ros2seminar2024/)用のROS 2パッケージです。

## 動作環境

- Linux OS
  - Ubuntu 22.04
- ROS
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## インストール方法

下記のコマンドを順に実行してください。

```sh
# Setup ROS 2 environment
source /opt/ros/humble/setup.bash

# Download urdf_seminar repositories
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rt-education/urdf_seminar.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## 動作方法

### RVizの起動

```sh
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_seminar crane_plus_rviz.launch.py
# Press [Ctrl-c] to terminate.
```

joint_state_publisher_guiのスライドバーを動かすと各関節の動作を確認できます。

### Gazeboの起動

```sh
# 1つ目のターミナル
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_seminar crane_plus_gazebo.launch.py
# Press [Ctrl-c] to terminate.
```

次のコマンドを実行すると各関節の角度を指定できます。
```
# 2つ目のターミナル
ros2 topic pub -1 /crane_plus_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5,0.5,0.5,0.5]}"
```

## ライセンス

(C) 2024 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。
