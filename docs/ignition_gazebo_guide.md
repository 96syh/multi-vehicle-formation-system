# Ignition Gazebo 编译和运行指南

## 概述

本指南将帮助您使用Ignition Gazebo（现称为Gazebo）运行多车编队仿真系统。

## 环境要求

### 系统要求
- Ubuntu 20.04 / 22.04
- ROS2 Humble / Galactic
- Ignition Gazebo Garden / Fortress

### 依赖安装

#### 1. 安装ROS2
```bash
# Ubuntu 22.04 - ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. 安装Ignition Gazebo
```bash
# 安装Ignition Gazebo Garden
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

#### 3. 安装ROS-Gazebo桥接包
```bash
# 安装ROS2-Ignition Gazebo桥接包
sudo apt install ros-humble-ros-gz
```

#### 4. 安装其他依赖
```bash
# 安装构建工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update

# 安装Eigen3
sudo apt install libeigen3-dev
```

## 编译项目

### 方法1: 使用自动化脚本 (推荐)
```bash
# 进入项目目录
cd multi_vehicle_formation_system

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 运行自动化脚本
./scripts/build_and_run_ignition.sh
```

### 方法2: 手动编译
```bash
# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 进入工作空间
cd ros2_workspace

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置工作空间环境
source install/setup.bash
```

## 运行仿真

### 基本运行
```bash
# 启动4车直线编队仿真
ros2 launch multi_vehicle_formation simulation.launch.py

# 指定参数
ros2 launch multi_vehicle_formation simulation.launch.py \
    num_vehicles:=6 \
    formation_type:=v_shape \
    gui:=true \
    rviz:=true
```

### 支持的编队类型
- `line`: 直线编队
- `v_shape`: V字形编队
- `diamond`: 菱形编队
- `circle`: 圆形编队

### 使用脚本快速启动
```bash
# 基本启动
./scripts/build_and_run_ignition.sh

# 指定参数
./scripts/build_and_run_ignition.sh -n 6 -f v_shape

# 无GUI模式
./scripts/build_and_run_ignition.sh --no-gui --no-rviz

# 查看帮助
./scripts/build_and_run_ignition.sh --help
```

## 查看仿真效果

### 1. Ignition Gazebo界面
- 3D仿真环境
- 车辆模型和运动
- 障碍物交互

### 2. RViz可视化
- 编队轨迹
- 传感器数据
- TF坐标系

### 3. 命令行监控
```bash
# 查看ROS2话题
ros2 topic list

# 监控编队状态
ros2 topic echo /formation_status

# 查看车辆里程计
ros2 topic echo /vehicle_0/odom
```

## 常见问题排查

### 1. 编译错误
```bash
# 检查依赖
rosdep check --from-paths src --ignore-src

# 重新安装依赖
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

### 2. Gazebo启动失败
```bash
# 检查Ignition Gazebo版本
gz --version

# 检查ROS-Gazebo桥接
ros2 pkg list | grep ros_gz
```

### 3. 节点通信问题
```bash
# 检查ROS域ID
echo $ROS_DOMAIN_ID

# 检查节点运行状态
ros2 node list
```

## 性能优化

### 1. 仿真参数调整
编辑 `config/formation_params.yaml`:
```yaml
simulation:
  real_time_factor: 1.0
  max_step_size: 0.001
  physics_engine: "bullet"
```

### 2. 渲染优化
```bash
# 设置渲染引擎
export IGNITION_GAZEBO_RENDER_ENGINE=ogre2

# 禁用阴影提升性能
export IGNITION_GAZEBO_DISABLE_SHADOWS=1
```

## 开发调试

### 1. 单独测试组件
```bash
# 仅启动Gazebo
gz sim worlds/formation_world.sdf

# 仅启动控制器
ros2 run multi_vehicle_formation formation_controller_node
```

### 2. 日志和调试
```bash
# 设置日志级别
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 查看详细输出
ros2 launch multi_vehicle_formation simulation.launch.py verbose:=true
```

## 扩展功能

### 1. 添加新的编队类型
编辑 `src/formation_algorithm.cpp`中的FormationGenerator类

### 2. 自定义车辆模型
修改 `worlds/formation_world.sdf`中的车辆定义

### 3. 添加传感器
在SDF文件中添加激光雷达、摄像头等传感器模型

## 支持和贡献

如果遇到问题或有改进建议，请：
1. 查看项目文档
2. 提交Issue或Pull Request
3. 参与社区讨论

---

**注意**: 确保所有依赖都正确安装，特别是ROS2和Ignition Gazebo的版本兼容性。 