# TurtleBot3 餐厅送餐机器人

TurtleBot3 Waffle 餐厅送餐系统，支持 ROS 1 Noetic 真机与 ROS 2 Humble 仿真。

## 项目结构

```
final-project/
├── maps/                    # 地图文件（已包含 restaurant_map）
├── catkin_ws/               # ROS 1 工作空间
│   └── src/turtlebot3_delivery/
├── docker_noetic/           # Docker 建图环境（Ubuntu 20.04 + ROS Noetic）
├── gui.py                    # ROS 2 送餐 GUI（Nav2）
├── requirements.txt         # Python 依赖
├── 地图信息.md              # 地图说明
└── 真机建图步骤.md          # 建图流程
```

## 快速开始

### ROS 1 真机送餐（默认使用项目 maps/）

```bash
cd catkin_ws && catkin_make && source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_delivery delivery_fsm.launch
```

### ROS 2 仿真 + GUI

```bash
pip install -r requirements.txt
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true slam:=false localization:=true rviz:=true \
  map:=$(pwd)/maps/restaurant_map.yaml
# 另开终端
python3 gui.py
```

## 依赖

- **ROS 1**：Ubuntu 20.04 + ROS Noetic（真机建图/导航）
- **ROS 2**：Ubuntu 22.04 + ROS 2 Humble（仿真、GUI）
- **Python**：`pip install -r requirements.txt`

## 文档

| 文档 | 说明 |
|------|------|
| [真机建图步骤.md](真机建图步骤.md) | 真机建图流程 |
| [从零设置地图.md](从零设置地图.md) | 建图与目标点配置 |
| [地图信息.md](地图信息.md) | 当前地图参数 |
| [docker_noetic/README.md](docker_noetic/README.md) | Docker 建图环境 |
