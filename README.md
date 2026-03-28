# Elfin ROS2 Visualizer Workspace

这是一个可直接构建的 ROS2 Humble 工作区，面向 Elfin 机械臂的可视化、MoveIt 仿真和基础控制。

仓库已经整理成标准工作区结构，克隆后进入仓库根目录即可直接 `colcon build`。

## 1. 能实现什么

- 构建 Elfin ROS2 全套基础包
- 启动 Elfin 机械臂的 RViz 可视化
- 用滑块控制关节角并发布 `joint_states`
- 在控制面板中切换机械臂型号
- 使用仓库内已有的 MoveIt2 / Gazebo / EtherCAT 相关包

这次额外加入了几项稳定性修复：

- RViz 直接读取生成的 URDF 文件，不再依赖 `/robot_description` 的启动时序
- `joint_states` 发布从 Tk GUI 事件循环中解耦，拖动滑块时更稳定
- 切换机械臂型号时会自动重启整条可视化链路
- 启动前自动清理旧的 `rviz2`、`robot_state_publisher`、`joint_state_publisher`

## 2. 目录结构

```text
elfin-ros2-visualizer-tools/
  control_elfin.py
  elfin_visualize.launch.py
  elfin_visualize.rviz
  run_elfin_visualize.sh
  src/
    elfin_robot_ros2/
```

- 根目录 4 个文件是这次补充的可视化入口
- `src/elfin_robot_ros2` 是完整的 Elfin ROS2 源码

## 3. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- `colcon`
- 图形桌面环境，用于 `rviz2` 和 Tk 控制面板

建议先安装：

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-joint-trajectory-controller \
  ros-humble-controller-manager \
  ros-humble-trajectory-msgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-position-controllers \
  ros-humble-moveit \
  build-essential \
  libgtk-3-dev \
  python3-pip
pip3 install transforms3d wxpython
```

## 4. 构建方法

```bash
git clone https://github.com/xiepm/elfin-ros2-visualizer-tools.git
cd elfin-ros2-visualizer-tools
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

我已经在仓库根目录执行过一次 `colcon build`，可以正常完成。

## 5. 可视化快速使用

以 `elfin5` 为例：

```bash
cd elfin-ros2-visualizer-tools
source /opt/ros/humble/setup.bash
source install/setup.bash
bash run_elfin_visualize.sh elfin5
```

支持型号：

- `elfin3`
- `elfin5`
- `elfin5_l`
- `elfin10`
- `elfin10_l`
- `elfin15`

使用说明：

- 机械臂型号请在控制面板顶部下拉框切换
- 不要手动点击 RViz 中的 `Description File` 去改模型路径
- 临时生成的 URDF 和 RViz 配置文件会放在 `/tmp/`

## 6. 仓库架构说明

详细架构文档见：

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

这里给一个简版：

- `run_elfin_visualize.sh`
  负责清理旧进程、加载 Humble 与工作区环境、启动统一 launch
- `elfin_visualize.launch.py`
  负责按型号生成临时 URDF、启动 `robot_state_publisher`、控制面板和 RViz
- `control_elfin.py`
  提供 Tk 控制面板，周期发布 `joint_states`，并在切换型号时重启整条链路
- `src/elfin_robot_ros2/elfin_description`
  提供 URDF/Xacro、网格模型和基础显示配置
- `src/elfin_robot_ros2/*_moveit2`
  提供各型号 MoveIt2 配置
- `src/elfin_robot_ros2/*_gazebo`
  提供各型号 Gazebo 配置
- `src/elfin_robot_ros2/elfin_basic_api`
  提供基础控制接口和 GUI 相关能力
- `src/elfin_robot_ros2/elfin_ethercat_driver`
  提供 EtherCAT 硬件驱动
- `src/elfin_robot_ros2/elfin_ros_control`
  提供 ros2_control 硬件接口

## 7. 上游源码说明

`src/elfin_robot_ros2` 来自 Elfin ROS2 源码，并在此基础上加入了 Humble 工作区整理和可视化修复入口。

源内原始说明文档已更新为 Humble 视角，可参考：

- [src/elfin_robot_ros2/README.md](src/elfin_robot_ros2/README.md)
- [src/elfin_robot_ros2/README_cn.md](src/elfin_robot_ros2/README_cn.md)
