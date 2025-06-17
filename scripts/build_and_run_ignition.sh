#!/bin/bash

# 多车编队系统 - Ignition Gazebo 编译和运行脚本
# Multi-Vehicle Formation System - Ignition Gazebo Build and Run Script

set -e  # 出错时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  多车编队系统 - Ignition Gazebo 版本${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未设置！请先source ROS2 setup.bash${NC}"
    echo -e "${YELLOW}例如: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 检测到ROS2环境: $ROS_DISTRO${NC}"

# 检查Ignition Gazebo
if ! command -v gz &> /dev/null; then
    echo -e "${RED}错误: Ignition Gazebo未安装！${NC}"
    echo -e "${YELLOW}请安装: sudo apt install gz-garden${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 检测到Ignition Gazebo${NC}"

# 检查依赖包
REQUIRED_PACKAGES=(
    "ros_gz_sim"
    "ros_gz_bridge"
    "ros_gz_interfaces"
)

for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! ros2 pkg list | grep -q "$package"; then
        echo -e "${RED}错误: 缺少ROS2包: $package${NC}"
        echo -e "${YELLOW}请安装: sudo apt install ros-$ROS_DISTRO-ros-gz${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓ 所有依赖包已安装${NC}"

# 进入工作空间
SCRIPT_DIR=$(dirname $(realpath $0))
WORKSPACE_DIR=$(dirname $SCRIPT_DIR)/ros2_workspace

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}错误: 工作空间目录不存在: $WORKSPACE_DIR${NC}"
    exit 1
fi

cd $WORKSPACE_DIR
echo -e "${GREEN}✓ 进入工作空间: $WORKSPACE_DIR${NC}"

# 清理之前的构建
echo -e "${YELLOW}清理之前的构建...${NC}"
rm -rf build/ install/ log/

# 安装依赖
echo -e "${YELLOW}安装ROS2依赖...${NC}"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译工作空间
echo -e "${YELLOW}编译ROS2工作空间...${NC}"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查编译结果
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 编译成功！${NC}"
else
    echo -e "${RED}✗ 编译失败！${NC}"
    exit 1
fi

# Source 工作空间
source install/setup.bash
echo -e "${GREEN}✓ 工作空间环境已设置${NC}"

# 解析命令行参数
NUM_VEHICLES=4
FORMATION_TYPE="line"
GUI=true
RVIZ=true

while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--vehicles)
            NUM_VEHICLES="$2"
            shift 2
            ;;
        -f|--formation)
            FORMATION_TYPE="$2"
            shift 2
            ;;
        --no-gui)
            GUI=false
            shift
            ;;
        --no-rviz)
            RVIZ=false
            shift
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  -n, --vehicles NUM     车辆数量 (默认: 4)"
            echo "  -f, --formation TYPE   编队类型 (line/v_shape/diamond/circle)"
            echo "  --no-gui              不启动Gazebo GUI"
            echo "  --no-rviz             不启动RViz"
            echo "  -h, --help            显示帮助信息"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}启动参数:${NC}"
echo -e "  车辆数量: $NUM_VEHICLES"
echo -e "  编队类型: $FORMATION_TYPE"
echo -e "  Gazebo GUI: $GUI"
echo -e "  RViz: $RVIZ"

# 启动仿真
echo -e "${YELLOW}启动多车编队仿真...${NC}"
ros2 launch multi_vehicle_formation simulation.launch.py \
    num_vehicles:=$NUM_VEHICLES \
    formation_type:=$FORMATION_TYPE \
    gui:=$GUI \
    rviz:=$RVIZ

echo -e "${GREEN}仿真结束${NC}" 