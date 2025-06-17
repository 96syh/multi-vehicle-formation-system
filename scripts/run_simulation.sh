#!/bin/bash
# 多车编队仿真快速启动脚本
# Multi-Vehicle Formation Simulation Quick Launch Script

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${GREEN}======================================"
echo -e "多车编队仿真系统启动"
echo -e "Multi-Vehicle Formation Simulation"
echo -e "======================================${NC}"

# 默认参数
NUM_VEHICLES=4
FORMATION_TYPE="line"
WORLD_FILE="formation_world.world"
USE_GUI=true
USE_RVIZ=true
RECORD_DATA=false

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--num-vehicles)
            NUM_VEHICLES="$2"
            shift 2
            ;;
        -f|--formation)
            FORMATION_TYPE="$2"
            shift 2
            ;;
        -w|--world)
            WORLD_file="$2"
            shift 2
            ;;
        --no-gui)
            USE_GUI=false
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --record)
            RECORD_DATA=true
            shift
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  -n, --num-vehicles NUM    车辆数量 (默认: 4)"
            echo "  -f, --formation TYPE      编队类型: line, v_shape, diamond, circle (默认: line)"
            echo "  -w, --world FILE          世界文件名 (默认: formation_world.world)"
            echo "  --no-gui                  不启动Gazebo GUI"
            echo "  --no-rviz                 不启动RViz可视化"
            echo "  --record                  启用数据记录"
            echo "  -h, --help                显示帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                                    # 使用默认参数启动"
            echo "  $0 -n 6 -f circle                   # 6辆车圆形编队"
            echo "  $0 -f v_shape --no-gui --record     # V形编队，无GUI，记录数据"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            echo "使用 -h 或 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 显示配置信息
echo -e "${YELLOW}仿真配置:${NC}"
echo "  车辆数量: $NUM_VEHICLES"
echo "  编队类型: $FORMATION_TYPE"
echo "  世界文件: $WORLD_FILE"
echo "  Gazebo GUI: $USE_GUI"
echo "  RViz可视化: $USE_RVIZ"
echo "  数据记录: $RECORD_DATA"
echo ""

# 检查环境
check_environment() {
    echo -e "${YELLOW}检查环境...${NC}"
    
    # 检查ROS2
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到ROS2。请确保已安装ROS2并source了环境。${NC}"
        exit 1
    fi
    
    # 检查工作空间
    if [ ! -f "$PROJECT_ROOT/ros2_workspace/install/setup.bash" ]; then
        echo -e "${RED}错误: ROS2工作空间未构建。请先运行 ./scripts/setup_environment.sh${NC}"
        exit 1
    fi
    
    # 检查Gazebo
    if ! command -v gazebo &> /dev/null; then
        echo -e "${YELLOW}警告: 未找到Gazebo。仿真可能无法正常运行。${NC}"
    fi
    
    echo -e "${GREEN}环境检查完成${NC}"
}

# 激活环境
activate_environment() {
    echo -e "${YELLOW}激活ROS2环境...${NC}"
    
    # Source ROS2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/galactic/setup.bash" ]; then
        source /opt/ros/galactic/setup.bash
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
    else
        echo -e "${RED}错误: 未找到ROS2安装${NC}"
        exit 1
    fi
    
    # Source workspace
    source "$PROJECT_ROOT/ros2_workspace/install/setup.bash"
    
    # Source project environment
    if [ -f "$PROJECT_ROOT/.env" ]; then
        source "$PROJECT_ROOT/.env"
    fi
    
    echo -e "${GREEN}环境激活完成${NC}"
}

# 启动仿真
launch_simulation() {
    echo -e "${YELLOW}启动仿真...${NC}"
    
    cd "$PROJECT_ROOT"
    
    # 构建启动参数
    LAUNCH_ARGS=""
    LAUNCH_ARGS+="num_vehicles:=$NUM_VEHICLES "
    LAUNCH_ARGS+="formation_type:=$FORMATION_TYPE "
    LAUNCH_ARGS+="world_file:=$WORLD_FILE "
    LAUNCH_ARGS+="gui:=$USE_GUI "
    LAUNCH_ARGS+="rviz:=$USE_RVIZ "
    LAUNCH_ARGS+="record_data:=$RECORD_DATA"
    
    echo -e "${GREEN}启动命令:${NC}"
    echo "ros2 launch multi_vehicle_formation simulation.launch.py $LAUNCH_ARGS"
    echo ""
    
    # 启动仿真
    ros2 launch multi_vehicle_formation simulation.launch.py $LAUNCH_ARGS
}

# 清理函数
cleanup() {
    echo -e "\n${YELLOW}正在清理...${NC}"
    
    # 杀死可能残留的进程
    pkill -f "gazebo" || true
    pkill -f "rviz" || true
    pkill -f "multi_vehicle_formation" || true
    
    echo -e "${GREEN}清理完成${NC}"
}

# 设置信号处理
trap cleanup EXIT INT TERM

# 主函数
main() {
    check_environment
    activate_environment
    launch_simulation
}

# 运行主函数
main "$@" 