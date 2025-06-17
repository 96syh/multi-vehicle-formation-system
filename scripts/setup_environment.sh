#!/bin/bash
# 多车编队系统环境配置脚本
# Multi-Vehicle Formation System Environment Setup Script

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

log_info "项目根目录: $PROJECT_ROOT"
log_info "开始配置多车编队系统环境..."

# 检查操作系统
check_os() {
    log_step "检查操作系统..."
    
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS="linux"
        log_info "检测到Linux系统"
        
        # 检查发行版
        if [ -f /etc/os-release ]; then
            . /etc/os-release
            DISTRO=$ID
            VERSION=$VERSION_ID
            log_info "发行版: $DISTRO $VERSION"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS="macos"
        log_info "检测到macOS系统"
    else
        log_error "不支持的操作系统: $OSTYPE"
        exit 1
    fi
}

# 检查ROS2安装
check_ros2() {
    log_step "检查ROS2安装..."
    
    if ! command -v ros2 &> /dev/null; then
        log_error "未检测到ROS2安装"
        log_info "请按照以下链接安装ROS2:"
        log_info "https://docs.ros.org/en/humble/Installation.html"
        
        if [[ "$OS" == "linux" ]]; then
            log_info "对于Ubuntu 22.04，可以运行以下命令安装ROS2 Humble:"
            echo "sudo apt update && sudo apt install software-properties-common"
            echo "sudo add-apt-repository universe"
            echo "sudo apt update && sudo apt install curl -y"
            echo "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
            echo "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
            echo "sudo apt update && sudo apt upgrade -y"
            echo "sudo apt install ros-humble-desktop -y"
        fi
        exit 1
    else
        ROS_DISTRO=$(ros2 --version | grep -oP '(?<=ros2 )\w+')
        log_info "检测到ROS2 $ROS_DISTRO"
    fi
}

# 安装系统依赖
install_system_dependencies() {
    log_step "安装系统依赖..."
    
    if [[ "$OS" == "linux" ]]; then
        log_info "更新包管理器..."
        sudo apt update
        
        log_info "安装构建工具..."
        sudo apt install -y \
            build-essential \
            cmake \
            git \
            python3-pip \
            python3-setuptools \
            python3-wheel \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-vcstool
        
        log_info "安装ROS2相关包..."
        sudo apt install -y \
            ros-$ROS_DISTRO-gazebo-ros-pkgs \
            ros-$ROS_DISTRO-navigation2 \
            ros-$ROS_DISTRO-nav2-bringup \
            ros-$ROS_DISTRO-robot-state-publisher \
            ros-$ROS_DISTRO-joint-state-publisher \
            ros-$ROS_DISTRO-rviz2 \
            ros-$ROS_DISTRO-tf2-tools \
            ros-$ROS_DISTRO-eigen3-cmake-module
        
        log_info "安装科学计算库..."
        sudo apt install -y \
            libeigen3-dev \
            libopencv-dev \
            python3-numpy \
            python3-scipy \
            python3-matplotlib
            
    elif [[ "$OS" == "macos" ]]; then
        if ! command -v brew &> /dev/null; then
            log_error "需要安装Homebrew"
            log_info "请运行: /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
            exit 1
        fi
        
        log_info "安装macOS依赖..."
        brew install cmake eigen opencv python@3.9
        pip3 install numpy scipy matplotlib
    fi
}

# 安装Python依赖
install_python_dependencies() {
    log_step "安装Python依赖..."
    
    # 创建requirements.txt如果不存在
    REQUIREMENTS_FILE="$PROJECT_ROOT/requirements.txt"
    if [ ! -f "$REQUIREMENTS_FILE" ]; then
        log_info "创建requirements.txt文件..."
        cat > "$REQUIREMENTS_FILE" << EOF
# 数据科学库
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0
pandas>=1.3.0

# 机器学习库
scikit-learn>=1.0.0

# 可视化库
seaborn>=0.11.0
plotly>=5.0.0

# 仿真和控制库
control>=0.9.0

# 参数优化库
scikit-optimize>=0.9.0

# YAML配置文件解析
PyYAML>=5.4.0

# ROS2 Python API
rclpy

# 数据处理
h5py>=3.6.0

# 串口通信（用于硬件接口）
pyserial>=3.5

# 网络通信
netifaces>=0.11.0

# 多进程
multiprocessing-logging>=0.3.0
EOF
    fi
    
    log_info "安装Python包..."
    pip3 install -r "$REQUIREMENTS_FILE"
}

# 初始化rosdep
initialize_rosdep() {
    log_step "初始化rosdep..."
    
    if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
        log_info "初始化rosdep..."
        sudo rosdep init
    else
        log_info "rosdep已初始化"
    fi
    
    log_info "更新rosdep数据库..."
    rosdep update
}

# 设置工作空间
setup_workspace() {
    log_step "设置ROS2工作空间..."
    
    WORKSPACE_DIR="$PROJECT_ROOT/ros2_workspace"
    
    if [ ! -d "$WORKSPACE_DIR" ]; then
        log_error "ROS2工作空间目录不存在: $WORKSPACE_DIR"
        exit 1
    fi
    
    cd "$WORKSPACE_DIR"
    
    log_info "安装工作空间依赖..."
    rosdep install --from-paths src --ignore-src -r -y
    
    log_info "构建工作空间..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        log_info "工作空间构建成功"
    else
        log_error "工作空间构建失败"
        exit 1
    fi
}

# 创建环境配置文件
create_env_config() {
    log_step "创建环境配置文件..."
    
    ENV_CONFIG_FILE="$PROJECT_ROOT/.env"
    cat > "$ENV_CONFIG_FILE" << EOF
# 多车编队系统环境配置文件
# Multi-Vehicle Formation System Environment Configuration

# ROS2配置
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Gazebo配置
export GAZEBO_MODEL_PATH=$PROJECT_ROOT/ros2_workspace/src/multi_vehicle_formation/models:\$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$PROJECT_ROOT/ros2_workspace/src/multi_vehicle_formation/worlds:\$GAZEBO_RESOURCE_PATH

# Python路径
export PYTHONPATH=$PROJECT_ROOT/simulation_tools/python_tools:\$PYTHONPATH

# 项目配置
export FORMATION_PROJECT_ROOT=$PROJECT_ROOT
export FORMATION_CONFIG_PATH=$PROJECT_ROOT/ros2_workspace/src/multi_vehicle_formation/config

# 日志配置
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
EOF
    
    log_info "环境配置文件已创建: $ENV_CONFIG_FILE"
}

# 创建激活脚本
create_activation_script() {
    log_step "创建环境激活脚本..."
    
    ACTIVATE_SCRIPT="$PROJECT_ROOT/activate_env.sh"
    cat > "$ACTIVATE_SCRIPT" << EOF
#!/bin/bash
# 多车编队系统环境激活脚本

# 获取脚本目录
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"

# 加载ROS2环境
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# 加载工作空间环境
if [ -f "\$SCRIPT_DIR/ros2_workspace/install/setup.bash" ]; then
    source \$SCRIPT_DIR/ros2_workspace/install/setup.bash
fi

# 加载项目环境变量
if [ -f "\$SCRIPT_DIR/.env" ]; then
    source \$SCRIPT_DIR/.env
fi

echo "多车编队系统环境已激活"
echo "项目根目录: \$FORMATION_PROJECT_ROOT"
echo "ROS2发行版: $ROS_DISTRO"
echo ""
echo "可用命令:"
echo "  ros2 launch multi_vehicle_formation simulation.launch.py  # 启动仿真"
echo "  ros2 run multi_vehicle_formation formation_controller_node  # 运行编队控制器"
echo "  ./scripts/run_simulation.sh  # 快速启动仿真"
echo ""
EOF
    
    chmod +x "$ACTIVATE_SCRIPT"
    log_info "环境激活脚本已创建: $ACTIVATE_SCRIPT"
}

# 验证安装
verify_installation() {
    log_step "验证安装..."
    
    # 检查ROS2包是否正确构建
    source "$PROJECT_ROOT/ros2_workspace/install/setup.bash"
    
    if ros2 pkg list | grep -q "multi_vehicle_formation"; then
        log_info "✓ ROS2包构建成功"
    else
        log_error "✗ ROS2包构建失败"
    fi
    
    # 检查Python依赖
    if python3 -c "import numpy, scipy, matplotlib" 2>/dev/null; then
        log_info "✓ Python依赖安装成功"
    else
        log_error "✗ Python依赖安装失败"
    fi
    
    # 检查Gazebo
    if command -v gazebo &> /dev/null; then
        log_info "✓ Gazebo可用"
    else
        log_warn "✗ Gazebo未安装或不可用"
    fi
}

# 主函数
main() {
    log_info "=================="
    log_info "环境配置开始"
    log_info "=================="
    
    check_os
    check_ros2
    install_system_dependencies
    install_python_dependencies
    initialize_rosdep
    setup_workspace
    create_env_config
    create_activation_script
    verify_installation
    
    log_info "=================="
    log_info "环境配置完成！"
    log_info "=================="
    
    log_info "使用方法:"
    log_info "1. 激活环境: source activate_env.sh"
    log_info "2. 运行仿真: ./scripts/run_simulation.sh"
    log_info "3. 查看文档: 参考 docs/ 目录下的文档"
}

# 运行主函数
main "$@" 