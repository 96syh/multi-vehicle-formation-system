# Multi-Vehicle Formation Control System

多车编队控制与仿真实验项目，包含一个可独立运行的 Python 编队仿真，以及面向 ROS2/Gazebo 和嵌入式控制的工程目录。项目用于验证多智能体队形保持、目标跟踪、避障和队形切换等控制策略。

## 功能概览

- 支持直线、V 形、菱形、圆形等常见队形。
- 提供基于人工势场、邻车斥力和队形约束的编队控制逻辑。
- 内置 matplotlib 可视化界面，便于观察车辆轨迹、目标点和队形误差。
- 包含 ROS2 工作区目录，可作为 Gazebo 联合仿真的扩展入口。
- 包含嵌入式系统目录，用于后续迁移到 STM32 等硬件平台。

## 快速运行

```bash
python -m pip install -r requirements.txt
python run_formation_demo.py
```

也可以直接指定车辆数量和队形：

```bash
python formation_simulation_standalone.py -n 6 -f v_shape --auto-start
```

## 目录结构

```text
.
├── formation_simulation_standalone.py   # 独立编队仿真主程序
├── run_formation_demo.py                # 交互式演示入口
├── requirements.txt                     # Python 最小依赖
├── docs/                                # 算法、参数和 Gazebo 说明
├── ros2_workspace/                      # ROS2 集成实验目录
├── embedded_system/                     # 嵌入式迁移相关代码
├── scripts/                             # 构建和启动脚本
└── simulation_tools/                    # 仿真辅助工具
```

## 运行环境

- Python 3.8+
- numpy
- matplotlib
- ROS2 Humble/Gazebo 相关功能为可选扩展

## 当前状态

独立 Python 仿真是主要可运行入口。ROS2/Gazebo 和嵌入式目录更偏工程扩展骨架，使用前需要结合本机 ROS2 环境、Gazebo 版本和硬件目标进行适配。

