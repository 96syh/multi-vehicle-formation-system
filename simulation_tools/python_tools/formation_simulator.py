#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多车编队仿真器
Multi-Vehicle Formation Simulator

这个模块提供了完整的多车编队仿真功能，包括：
- 编队控制算法仿真
- 动态队形变换
- 避障算法测试
- 性能评估和可视化

作者: Formation Control Team
版本: 1.0
日期: 2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Polygon
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
import yaml
import logging
from pathlib import Path
import time
import json

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class VehicleState:
    """车辆状态类"""
    id: int
    position: np.ndarray  # [x, y]
    velocity: np.ndarray  # [vx, vy]
    heading: float        # 航向角 (rad)
    angular_velocity: float = 0.0
    timestamp: float = 0.0
    is_valid: bool = True
    is_leader: bool = False

@dataclass
class Obstacle:
    """障碍物类"""
    position: np.ndarray  # [x, y]
    radius: float
    is_dynamic: bool = False
    velocity: np.ndarray = None  # [vx, vy] for dynamic obstacles
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = np.zeros(2)

@dataclass
class ControlParams:
    """控制参数类"""
    # 人工势场参数
    k_att: float = 1.0          # 吸引力增益
    k_rep: float = 2.0          # 排斥力增益
    k_form: float = 1.5         # 编队力增益
    k_obstacle: float = 3.0     # 障碍物排斥力增益
    
    # 影响范围
    influence_radius: float = 3.0    # 势场影响半径
    obstacle_radius: float = 2.0     # 障碍物影响半径
    safety_distance: float = 1.0     # 安全距离
    
    # 运动限制
    max_velocity: float = 2.0        # 最大速度
    max_angular_velocity: float = 1.0 # 最大角速度
    max_acceleration: float = 1.0    # 最大加速度
    
    # 共识算法参数
    consensus_gain: float = 0.5      # 共识增益
    damping_factor: float = 0.8      # 阻尼系数
    convergence_threshold: float = 0.1 # 收敛阈值

class FormationGenerator:
    """编队生成器"""
    
    @staticmethod
    def generate_line_formation(num_vehicles: int, spacing: float = 2.0, 
                               angle: float = 0.0) -> np.ndarray:
        """生成直线编队"""
        positions = np.zeros((num_vehicles, 2))
        for i in range(num_vehicles):
            x = (i - (num_vehicles - 1) / 2) * spacing * np.cos(angle)
            y = (i - (num_vehicles - 1) / 2) * spacing * np.sin(angle)
            positions[i] = [x, y]
        return positions
    
    @staticmethod
    def generate_v_formation(num_vehicles: int, spacing: float = 2.0, 
                            angle: float = np.pi/4) -> np.ndarray:
        """生成V形编队"""
        positions = np.zeros((num_vehicles, 2))
        positions[0] = [0, 0]  # 领导者在顶点
        
        for i in range(1, num_vehicles):
            side = 1 if i % 2 == 1 else -1  # 左右交替
            row = (i + 1) // 2
            x = -row * spacing * np.cos(angle)
            y = side * row * spacing * np.sin(angle)
            positions[i] = [x, y]
        
        return positions
    
    @staticmethod
    def generate_circle_formation(num_vehicles: int, radius: float = 2.0) -> np.ndarray:
        """生成圆形编队"""
        positions = np.zeros((num_vehicles, 2))
        for i in range(num_vehicles):
            angle = 2 * np.pi * i / num_vehicles
            positions[i] = [radius * np.cos(angle), radius * np.sin(angle)]
        return positions
    
    @staticmethod
    def generate_diamond_formation(num_vehicles: int, size: float = 2.0) -> np.ndarray:
        """生成菱形编队"""
        if num_vehicles < 4:
            return FormationGenerator.generate_line_formation(num_vehicles)
        
        positions = np.zeros((num_vehicles, 2))
        
        # 菱形的四个顶点
        diamond_points = np.array([
            [0, size],      # 上
            [size, 0],      # 右
            [0, -size],     # 下
            [-size, 0]      # 左
        ])
        
        # 将车辆分配到菱形边上
        for i in range(min(4, num_vehicles)):
            positions[i] = diamond_points[i]
        
        # 如果车辆数超过4，在内部添加
        if num_vehicles > 4:
            inner_radius = size * 0.5
            for i in range(4, num_vehicles):
                angle = 2 * np.pi * (i - 4) / (num_vehicles - 4)
                positions[i] = [inner_radius * np.cos(angle), 
                               inner_radius * np.sin(angle)]
        
        return positions

class FormationController:
    """编队控制器"""
    
    def __init__(self, params: ControlParams = None):
        self.params = params or ControlParams()
        self.vehicles: Dict[int, VehicleState] = {}
        self.obstacles: List[Obstacle] = []
        self.target_formation: np.ndarray = None
        self.formation_center: np.ndarray = np.zeros(2)
        self.formation_heading: float = 0.0
        
        # 性能记录
        self.error_history: List[float] = []
        self.time_history: List[float] = []
        
    def add_vehicle(self, vehicle: VehicleState):
        """添加车辆"""
        self.vehicles[vehicle.id] = vehicle
        
    def add_obstacle(self, obstacle: Obstacle):
        """添加障碍物"""
        self.obstacles.append(obstacle)
        
    def set_formation(self, formation_type: str, num_vehicles: int = None):
        """设置编队类型"""
        if num_vehicles is None:
            num_vehicles = len(self.vehicles)
            
        if formation_type == "line":
            self.target_formation = FormationGenerator.generate_line_formation(num_vehicles)
        elif formation_type == "v_shape":
            self.target_formation = FormationGenerator.generate_v_formation(num_vehicles)
        elif formation_type == "circle":
            self.target_formation = FormationGenerator.generate_circle_formation(num_vehicles)
        elif formation_type == "diamond":
            self.target_formation = FormationGenerator.generate_diamond_formation(num_vehicles)
        else:
            raise ValueError(f"不支持的编队类型: {formation_type}")
            
        logger.info(f"设置编队类型: {formation_type}, 车辆数量: {num_vehicles}")
    
    def calculate_attractive_force(self, vehicle_id: int, target_pos: np.ndarray) -> np.ndarray:
        """计算吸引力"""
        vehicle = self.vehicles[vehicle_id]
        direction = target_pos - vehicle.position
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return np.zeros(2)
        
        # 吸引力与距离成正比
        force_magnitude = self.params.k_att * distance
        force_direction = direction / distance
        
        return force_magnitude * force_direction
    
    def calculate_repulsive_force(self, vehicle_id: int, repulsive_pos: np.ndarray) -> np.ndarray:
        """计算排斥力"""
        vehicle = self.vehicles[vehicle_id]
        direction = vehicle.position - repulsive_pos
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6 or distance > self.params.influence_radius:
            return np.zeros(2)
        
        # 排斥力与距离的平方成反比
        force_magnitude = self.params.k_rep * (1.0/distance - 1.0/self.params.influence_radius) / (distance * distance)
        force_direction = direction / distance
        
        return force_magnitude * force_direction
    
    def calculate_formation_force(self, vehicle_id: int) -> np.ndarray:
        """计算编队约束力"""
        if self.target_formation is None:
            return np.zeros(2)
        
        vehicle = self.vehicles[vehicle_id]
        
        # 获取期望位置
        if vehicle_id < len(self.target_formation):
            desired_global_pos = self.target_formation[vehicle_id] + self.formation_center
            
            # 考虑编队航向
            if abs(self.formation_heading) > 1e-6:
                rotation_matrix = np.array([
                    [np.cos(self.formation_heading), -np.sin(self.formation_heading)],
                    [np.sin(self.formation_heading), np.cos(self.formation_heading)]
                ])
                relative_pos = self.target_formation[vehicle_id]
                rotated_pos = rotation_matrix @ relative_pos
                desired_global_pos = rotated_pos + self.formation_center
            
            # 计算编队约束力
            return self.calculate_attractive_force(vehicle_id, desired_global_pos) * self.params.k_form
        
        return np.zeros(2)
    
    def calculate_obstacle_force(self, vehicle_id: int) -> np.ndarray:
        """计算障碍物排斥力"""
        vehicle = self.vehicles[vehicle_id]
        total_force = np.zeros(2)
        
        for obstacle in self.obstacles:
            direction = vehicle.position - obstacle.position
            distance = np.linalg.norm(direction) - obstacle.radius
            
            if distance < 1e-6 or distance > self.params.obstacle_radius:
                continue
            
            # 障碍物排斥力
            force_magnitude = self.params.k_obstacle * (1.0/distance - 1.0/self.params.obstacle_radius) / (distance * distance)
            force_direction = direction / np.linalg.norm(direction)
            
            total_force += force_magnitude * force_direction
        
        return total_force
    
    def calculate_consensus_force(self, vehicle_id: int) -> np.ndarray:
        """计算共识控制力"""
        vehicle = self.vehicles[vehicle_id]
        consensus_force = np.zeros(2)
        neighbor_count = 0
        
        for other_id, other_vehicle in self.vehicles.items():
            if other_id == vehicle_id or not other_vehicle.is_valid:
                continue
            
            distance = np.linalg.norm(other_vehicle.position - vehicle.position)
            if distance < self.params.influence_radius:
                # 获取期望相对位置
                if (vehicle_id < len(self.target_formation) and 
                    other_id < len(self.target_formation)):
                    desired_relative = (self.target_formation[other_id] - 
                                      self.target_formation[vehicle_id])
                    actual_relative = other_vehicle.position - vehicle.position
                    error = actual_relative - desired_relative
                    
                    consensus_force += self.params.consensus_gain * error
                    neighbor_count += 1
        
        # 应用阻尼
        if neighbor_count > 0:
            consensus_force /= neighbor_count
            consensus_force *= self.params.damping_factor
        
        return consensus_force
    
    def calculate_total_force(self, vehicle_id: int) -> np.ndarray:
        """计算总合力"""
        formation_force = self.calculate_formation_force(vehicle_id)
        repulsive_force = np.zeros(2)
        
        # 计算与其他车辆的排斥力
        for other_id, other_vehicle in self.vehicles.items():
            if other_id != vehicle_id and other_vehicle.is_valid:
                repulsive_force += self.calculate_repulsive_force(vehicle_id, other_vehicle.position)
        
        # 计算障碍物排斥力
        obstacle_force = self.calculate_obstacle_force(vehicle_id)
        
        # 计算共识控制力
        consensus_force = self.calculate_consensus_force(vehicle_id)
        
        # 总合力
        total_force = formation_force + repulsive_force + obstacle_force + consensus_force
        
        return total_force
    
    def update_vehicle(self, vehicle_id: int, dt: float):
        """更新车辆状态"""
        if vehicle_id not in self.vehicles:
            return
        
        vehicle = self.vehicles[vehicle_id]
        if not vehicle.is_valid:
            return
        
        # 计算合力
        force = self.calculate_total_force(vehicle_id)
        
        # 限制加速度
        acceleration = force
        acc_magnitude = np.linalg.norm(acceleration)
        if acc_magnitude > self.params.max_acceleration:
            acceleration = acceleration / acc_magnitude * self.params.max_acceleration
        
        # 更新速度
        vehicle.velocity += acceleration * dt
        
        # 限制速度
        vel_magnitude = np.linalg.norm(vehicle.velocity)
        if vel_magnitude > self.params.max_velocity:
            vehicle.velocity = vehicle.velocity / vel_magnitude * self.params.max_velocity
        
        # 更新位置
        vehicle.position += vehicle.velocity * dt
        
        # 更新航向角
        if vel_magnitude > 0.1:  # 避免低速时的航向噪声
            desired_heading = np.arctan2(vehicle.velocity[1], vehicle.velocity[0])
            heading_error = desired_heading - vehicle.heading
            
            # 角度归一化
            while heading_error > np.pi:
                heading_error -= 2 * np.pi
            while heading_error < -np.pi:
                heading_error += 2 * np.pi
            
            # 限制角速度
            angular_velocity = heading_error * 2.0  # 简单的角度控制
            if abs(angular_velocity) > self.params.max_angular_velocity:
                angular_velocity = np.sign(angular_velocity) * self.params.max_angular_velocity
            
            vehicle.heading += angular_velocity * dt
            vehicle.angular_velocity = angular_velocity
        
        # 更新时间戳
        vehicle.timestamp = time.time()
    
    def update_all_vehicles(self, dt: float):
        """更新所有车辆状态"""
        for vehicle_id in self.vehicles.keys():
            self.update_vehicle(vehicle_id, dt)
    
    def calculate_formation_error(self) -> float:
        """计算编队误差"""
        if self.target_formation is None:
            return 0.0
        
        total_error = 0.0
        valid_vehicles = 0
        
        for vehicle_id, vehicle in self.vehicles.items():
            if not vehicle.is_valid or vehicle_id >= len(self.target_formation):
                continue
            
            desired_pos = self.target_formation[vehicle_id] + self.formation_center
            
            # 考虑编队航向
            if abs(self.formation_heading) > 1e-6:
                rotation_matrix = np.array([
                    [np.cos(self.formation_heading), -np.sin(self.formation_heading)],
                    [np.sin(self.formation_heading), np.cos(self.formation_heading)]
                ])
                relative_pos = self.target_formation[vehicle_id]
                rotated_pos = rotation_matrix @ relative_pos
                desired_pos = rotated_pos + self.formation_center
            
            error = np.linalg.norm(vehicle.position - desired_pos)
            total_error += error
            valid_vehicles += 1
        
        return total_error / max(valid_vehicles, 1)
    
    def is_converged(self) -> bool:
        """检查是否收敛"""
        return self.calculate_formation_error() < self.params.convergence_threshold

class FormationSimulator:
    """编队仿真器主类"""
    
    def __init__(self, config_file: str = None):
        self.controller = FormationController()
        self.time_step = 0.1
        self.simulation_time = 0.0
        self.max_simulation_time = 100.0
        
        # 可视化相关
        self.fig = None
        self.ax = None
        self.vehicle_plots = {}
        self.trajectory_plots = {}
        self.formation_lines = None
        self.obstacle_patches = []
        
        # 数据记录
        self.simulation_data = {
            'time': [],
            'vehicles': {},
            'formation_error': [],
            'converged': []
        }
        
        if config_file:
            self.load_config(config_file)
    
    def load_config(self, config_file: str):
        """加载配置文件"""
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # 加载控制参数
            if 'control_params' in config:
                params_dict = config['control_params']
                self.controller.params = ControlParams(**params_dict)
            
            # 加载仿真参数
            if 'simulation' in config:
                sim_config = config['simulation']
                self.time_step = sim_config.get('time_step', 0.1)
                self.max_simulation_time = sim_config.get('max_time', 100.0)
            
            logger.info(f"配置文件加载成功: {config_file}")
        
        except Exception as e:
            logger.error(f"配置文件加载失败: {e}")
    
    def add_vehicles(self, num_vehicles: int, initial_positions: np.ndarray = None):
        """添加车辆"""
        if initial_positions is None:
            # 随机初始位置
            initial_positions = np.random.uniform(-5, 5, (num_vehicles, 2))
        
        for i in range(num_vehicles):
            vehicle = VehicleState(
                id=i,
                position=initial_positions[i].copy(),
                velocity=np.zeros(2),
                heading=0.0,
                is_leader=(i == 0)
            )
            self.controller.add_vehicle(vehicle)
            
            # 初始化数据记录
            self.simulation_data['vehicles'][i] = {
                'positions': [],
                'velocities': [],
                'headings': []
            }
        
        logger.info(f"添加 {num_vehicles} 辆车辆")
    
    def add_obstacles(self, obstacles: List[Dict]):
        """添加障碍物"""
        for obs_config in obstacles:
            obstacle = Obstacle(
                position=np.array(obs_config['position']),
                radius=obs_config['radius'],
                is_dynamic=obs_config.get('is_dynamic', False),
                velocity=np.array(obs_config.get('velocity', [0, 0]))
            )
            self.controller.add_obstacle(obstacle)
        
        logger.info(f"添加 {len(obstacles)} 个障碍物")
    
    def set_formation(self, formation_type: str):
        """设置编队类型"""
        self.controller.set_formation(formation_type)
    
    def run_simulation(self, duration: float = None, enable_visualization: bool = True):
        """运行仿真"""
        if duration:
            self.max_simulation_time = duration
        
        logger.info("开始仿真...")
        
        if enable_visualization:
            self.setup_visualization()
            self.run_animated_simulation()
        else:
            self.run_headless_simulation()
        
        logger.info("仿真完成")
    
    def run_headless_simulation(self):
        """无界面仿真"""
        while self.simulation_time < self.max_simulation_time:
            # 更新车辆状态
            self.controller.update_all_vehicles(self.time_step)
            
            # 更新动态障碍物
            self.update_dynamic_obstacles()
            
            # 记录数据
            self.record_data()
            
            # 更新时间
            self.simulation_time += self.time_step
            
            # 检查收敛
            if self.controller.is_converged():
                logger.info(f"编队在 {self.simulation_time:.2f}s 时收敛")
                break
    
    def update_dynamic_obstacles(self):
        """更新动态障碍物"""
        for obstacle in self.controller.obstacles:
            if obstacle.is_dynamic:
                obstacle.position += obstacle.velocity * self.time_step
    
    def record_data(self):
        """记录仿真数据"""
        self.simulation_data['time'].append(self.simulation_time)
        
        for vehicle_id, vehicle in self.controller.vehicles.items():
            self.simulation_data['vehicles'][vehicle_id]['positions'].append(
                vehicle.position.copy())
            self.simulation_data['vehicles'][vehicle_id]['velocities'].append(
                vehicle.velocity.copy())
            self.simulation_data['vehicles'][vehicle_id]['headings'].append(
                vehicle.heading)
        
        formation_error = self.controller.calculate_formation_error()
        self.simulation_data['formation_error'].append(formation_error)
        self.simulation_data['converged'].append(self.controller.is_converged())
        
        # 更新误差历史
        self.controller.error_history.append(formation_error)
        self.controller.time_history.append(self.simulation_time)
    
    def setup_visualization(self):
        """设置可视化"""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('多车编队仿真 - Multi-Vehicle Formation Simulation')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        # 初始化车辆绘图对象
        for vehicle_id in self.controller.vehicles.keys():
            # 车辆位置
            vehicle_plot, = self.ax.plot([], [], 'o', markersize=8, 
                                       label=f'Vehicle {vehicle_id}')
            self.vehicle_plots[vehicle_id] = vehicle_plot
            
            # 轨迹线
            trajectory_plot, = self.ax.plot([], [], '-', alpha=0.5, linewidth=1)
            self.trajectory_plots[vehicle_id] = trajectory_plot
        
        # 绘制障碍物
        for obstacle in self.controller.obstacles:
            circle = Circle(obstacle.position, obstacle.radius, 
                          color='red', alpha=0.5, label='Obstacle')
            self.ax.add_patch(circle)
            self.obstacle_patches.append(circle)
        
        self.ax.legend()
        
    def run_animated_simulation(self):
        """运行动画仿真"""
        def animate(frame):
            # 更新仿真状态
            if self.simulation_time < self.max_simulation_time and not self.controller.is_converged():
                self.controller.update_all_vehicles(self.time_step)
                self.update_dynamic_obstacles()
                self.record_data()
                self.simulation_time += self.time_step
            
            # 更新可视化
            self.update_visualization()
            
            return list(self.vehicle_plots.values()) + list(self.trajectory_plots.values())
        
        ani = animation.FuncAnimation(self.fig, animate, interval=50, blit=False)
        plt.show()
    
    def update_visualization(self):
        """更新可视化"""
        for vehicle_id, vehicle in self.controller.vehicles.items():
            # 更新车辆位置
            self.vehicle_plots[vehicle_id].set_data([vehicle.position[0]], [vehicle.position[1]])
            
            # 更新轨迹
            if vehicle_id in self.simulation_data['vehicles']:
                positions = self.simulation_data['vehicles'][vehicle_id]['positions']
                if len(positions) > 1:
                    x_data = [pos[0] for pos in positions]
                    y_data = [pos[1] for pos in positions]
                    self.trajectory_plots[vehicle_id].set_data(x_data, y_data)
        
        # 更新编队连线
        if self.controller.target_formation is not None:
            self.update_formation_lines()
        
        # 更新动态障碍物
        for i, obstacle in enumerate(self.controller.obstacles):
            if obstacle.is_dynamic and i < len(self.obstacle_patches):
                self.obstacle_patches[i].center = obstacle.position
        
        # 更新标题显示当前状态
        error = self.controller.calculate_formation_error()
        converged = self.controller.is_converged()
        self.ax.set_title(f'多车编队仿真 - 时间: {self.simulation_time:.1f}s, '
                         f'误差: {error:.3f}m, 收敛: {"是" if converged else "否"}')
    
    def update_formation_lines(self):
        """更新编队连线显示"""
        # 移除旧的连线
        if self.formation_lines:
            for line in self.formation_lines:
                line.remove()
        
        self.formation_lines = []
        
        # 绘制期望编队形状
        if len(self.controller.target_formation) > 1:
            formation_x = self.controller.target_formation[:, 0] + self.controller.formation_center[0]
            formation_y = self.controller.target_formation[:, 1] + self.controller.formation_center[1]
            
            # 连接相邻点形成编队形状
            for i in range(len(formation_x)):
                for j in range(i + 1, len(formation_x)):
                    line, = self.ax.plot([formation_x[i], formation_x[j]], 
                                       [formation_y[i], formation_y[j]], 
                                       '--', color='gray', alpha=0.3)
                    self.formation_lines.append(line)
    
    def save_results(self, filename: str):
        """保存仿真结果"""
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(self.simulation_data, f, indent=2, default=str)
            logger.info(f"结果已保存到: {filename}")
        except Exception as e:
            logger.error(f"保存结果失败: {e}")
    
    def plot_performance(self):
        """绘制性能曲线"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # 编队误差曲线
        axes[0, 0].plot(self.simulation_data['time'], self.simulation_data['formation_error'])
        axes[0, 0].set_title('编队误差随时间变化')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('编队误差 (m)')
        axes[0, 0].grid(True)
        
        # 车辆轨迹图
        for vehicle_id, data in self.simulation_data['vehicles'].items():
            positions = np.array(data['positions'])
            if len(positions) > 0:
                axes[0, 1].plot(positions[:, 0], positions[:, 1], 
                              label=f'Vehicle {vehicle_id}')
        axes[0, 1].set_title('车辆轨迹')
        axes[0, 1].set_xlabel('X (m)')
        axes[0, 1].set_ylabel('Y (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        axes[0, 1].set_aspect('equal')
        
        # 速度变化
        for vehicle_id, data in self.simulation_data['vehicles'].items():
            velocities = np.array(data['velocities'])
            if len(velocities) > 0:
                speed = np.linalg.norm(velocities, axis=1)
                axes[1, 0].plot(self.simulation_data['time'][:len(speed)], speed, 
                              label=f'Vehicle {vehicle_id}')
        axes[1, 0].set_title('车辆速度变化')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('速度 (m/s)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 收敛性分析
        convergence_times = []
        for i, converged in enumerate(self.simulation_data['converged']):
            if converged:
                convergence_times.append(i)
        
        if convergence_times:
            convergence_time = self.simulation_data['time'][convergence_times[0]]
            axes[1, 1].axvline(x=convergence_time, color='red', linestyle='--', 
                             label=f'收敛时间: {convergence_time:.2f}s')
        
        axes[1, 1].plot(self.simulation_data['time'], self.simulation_data['formation_error'])
        axes[1, 1].set_title('收敛性分析')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('编队误差 (m)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()

# 示例使用
if __name__ == "__main__":
    # 创建仿真器
    simulator = FormationSimulator()
    
    # 添加车辆
    num_vehicles = 4
    initial_positions = np.array([
        [0, 0], [2, 0], [0, 2], [2, 2]
    ])
    simulator.add_vehicles(num_vehicles, initial_positions)
    
    # 添加障碍物
    obstacles = [
        {'position': [5, 5], 'radius': 1.0, 'is_dynamic': False},
        {'position': [-3, 3], 'radius': 0.8, 'is_dynamic': True, 'velocity': [0.5, -0.2]}
    ]
    simulator.add_obstacles(obstacles)
    
    # 设置编队类型
    simulator.set_formation("line")
    
    # 运行仿真
    simulator.run_simulation(duration=30.0, enable_visualization=True)
    
    # 绘制性能曲线
    simulator.plot_performance()
    
    # 保存结果
    simulator.save_results("simulation_results.json") 