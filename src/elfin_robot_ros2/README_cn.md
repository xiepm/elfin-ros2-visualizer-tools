Elfin Robot
======

英文版 README 请参考 [README.md](./README.md)

<p align="center">
  <img src="docs/images/elfin.png" />
</p>

这个工作区为 Elfin 机械臂提供 ROS2 Humble 支持，并已经整理成可直接 `colcon build` 的工作区结构。

### 安装

#### Ubuntu 22.04 + ROS2 Humble

**安装核心依赖：**
```sh
$ sudo apt-get install ros-humble-joint-trajectory-controller
$ sudo apt-get install ros-humble-controller-manager
$ sudo apt-get install ros-humble-trajectory-msgs
$ sudo apt-get install ros-humble-gazebo-ros2-control
$ sudo apt-get install ros-humble-joint-state-publisher
$ sudo apt-get install ros-humble-joint-state-publisher-gui
$ sudo apt-get install ros-humble-position-controllers
```

**安装相关软件包：**
```sh
$ sudo apt-get install build-essential libgtk-3-dev python3-pip
$ pip3 install wxpython
$ pip3 install transforms3d
```

**安装 MoveIt2：**
```sh
$ sudo apt-get update
$ sudo apt-get install ros-humble-moveit
```

**从源码构建整个工作区：**
```sh
$ git clone https://github.com/xiepm/elfin-ros2-visualizer-tools.git
$ cd elfin-ros2-visualizer-tools
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash
```

### 快速可视化使用

```sh
$ bash run_elfin_visualize.sh elfin5
```

支持型号：

- `elfin3`
- `elfin5`
- `elfin5_l`
- `elfin10`
- `elfin10_l`
- `elfin15`

### Gazebo 仿真

以 `elfin3` 为例：

```sh
$ ros2 launch elfin3_ros2_moveit2 elfin3.launch.py
```

启动 basic api 和控制界面：

```sh
$ ros2 launch elfin3_ros2_moveit2 elfin3_basic_api.launch.py
$ ros2 launch elfin_basic_api fake_elfin_gui.launch.py
```

MoveIt RViz 插件教程见：

- [docs/moveit_plugin_tutorial.md](docs/moveit_plugin_tutorial.md)

### 真机使用

真机相关路径主要涉及：

- `elfin_robot_bringup`
- `elfin_ethercat_driver`
- `elfin_ros_control`

使用前需要：

- 正确配置厂商提供的 EtherCAT 参数文件
- 正确配置网卡名
- 正确准备 PREEMPT_RT 环境

### 架构说明

根目录附带了一份架构文档，建议配合阅读：

- [/home/xpm/workspace/elfin-ros2-visualizer-tools/docs/ARCHITECTURE.md](/home/xpm/workspace/elfin-ros2-visualizer-tools/docs/ARCHITECTURE.md)
