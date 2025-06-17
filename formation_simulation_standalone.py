#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立多车编队仿真系统
Standalone Multi-Vehicle Formation Simulation System

这是一个完整的多车编队仿真器，尽量贴合Gazebo的物理特性
包含实时可视化、多种编队算法、避障功能和交互控制

作者: Formation Control Team
版本: 2.0
日期: 2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Arrow, Polygon
from matplotlib.widgets import Slider, Button, RadioButtons
import tkinter as tk
from tkinter import ttk
import threading
import time
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import argparse

# 设置matplotlib支持中文
plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

@dataclass
class VehicleState:
    """车辆状态类 - 贴合Gazebo物理模型"""
    id: int
    position: np.ndarray      # [x, y] 位置
    velocity: np.ndarray      # [vx, vy] 速度
    heading: float           # 航向角 (rad)
    angular_velocity: float  # 角速度 (rad/s)
    linear_velocity: float   # 线速度 (m/s)
    acceleration: np.ndarray # [ax, ay] 加速度
    
    # 物理参数
    mass: float = 10.0       # 质量 (kg)
    inertia: float = 0.5     # 转动惯量
    wheel_base: float = 0.3  # 轴距
    max_speed: float = 2.0   # 最大速度
    max_angular_speed: float = 1.0  # 最大角速度
    
    # 状态标志
    is_leader: bool = False
    is_active: bool = True
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(2)
        if self.velocity is None:
            self.velocity = np.zeros(2)
        if self.acceleration is None:
            self.acceleration = np.zeros(2)

@dataclass
class Obstacle:
    """障碍物类"""
    position: np.ndarray
    radius: float
    is_dynamic: bool = False
    velocity: np.ndarray = None
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = np.zeros(2)

class FormationController:
    """编队控制器 - 包含多种算法"""
    
    def __init__(self):
        # 控制参数
        self.k_att = 1.0          # 吸引力系数
        self.k_rep = 2.0          # 排斥力系数  
        self.k_form = 1.5         # 编队力系数
        self.k_obstacle = 3.0     # 障碍物排斥力系数
        self.k_damping = 0.8      # 阻尼系数
        
        # 影响范围
        self.influence_radius = 3.0
        self.obstacle_radius = 2.0
        self.safety_distance = 1.0
        
        # 编队参数
        self.formation_type = "line"
        self.formation_scale = 2.0
        self.formation_center = np.zeros(2)
        self.formation_heading = 0.0
        
        # 目标位置
        self.target_position = np.array([10.0, 5.0])
        
    def generate_formation_positions(self, num_vehicles: int) -> np.ndarray:
        """生成编队位置"""
        positions = np.zeros((num_vehicles, 2))
        
        if self.formation_type == "line":
            for i in range(num_vehicles):
                x = (i - (num_vehicles - 1) / 2) * self.formation_scale
                y = 0
                positions[i] = [x, y]
                
        elif self.formation_type == "v_shape":
            positions[0] = [0, 0]  # 领导者
            for i in range(1, num_vehicles):
                side = 1 if i % 2 == 1 else -1
                row = (i + 1) // 2
                x = -row * self.formation_scale * 0.8
                y = side * row * self.formation_scale * 0.6
                positions[i] = [x, y]
                
        elif self.formation_type == "diamond":
            if num_vehicles >= 4:
                positions[0] = [0, self.formation_scale]    # 上
                positions[1] = [self.formation_scale, 0]    # 右
                positions[2] = [0, -self.formation_scale]   # 下
                positions[3] = [-self.formation_scale, 0]   # 左
                
                # 额外车辆放在内部
                for i in range(4, num_vehicles):
                    angle = 2 * np.pi * (i - 4) / max(1, num_vehicles - 4)
                    r = self.formation_scale * 0.5
                    positions[i] = [r * np.cos(angle), r * np.sin(angle)]
            else:
                # 少于4辆车使用直线编队
                return self.generate_formation_positions(num_vehicles)
                
        elif self.formation_type == "circle":
            for i in range(num_vehicles):
                angle = 2 * np.pi * i / num_vehicles
                x = self.formation_scale * np.cos(angle)
                y = self.formation_scale * np.sin(angle)
                positions[i] = [x, y]
                
        # 应用编队中心和旋转
        rotation_matrix = np.array([
            [np.cos(self.formation_heading), -np.sin(self.formation_heading)],
            [np.sin(self.formation_heading), np.cos(self.formation_heading)]
        ])
        
        for i in range(num_vehicles):
            positions[i] = rotation_matrix @ positions[i] + self.formation_center
            
        return positions
    
    def calculate_formation_force(self, vehicle: VehicleState, 
                                desired_pos: np.ndarray, 
                                neighbors: List[VehicleState],
                                obstacles: List[Obstacle]) -> np.ndarray:
        """计算编队控制力"""
        total_force = np.zeros(2)
        
        # 1. 目标吸引力
        target_dir = self.target_position - vehicle.position
        target_dist = np.linalg.norm(target_dir)
        if target_dist > 0.1:
            target_force = self.k_att * target_dir / target_dist
            total_force += target_force
        
        # 2. 编队约束力
        formation_error = desired_pos - vehicle.position
        formation_force = self.k_form * formation_error
        total_force += formation_force
        
        # 3. 邻车排斥力
        for neighbor in neighbors:
            if neighbor.id != vehicle.id and neighbor.is_active:
                relative_pos = vehicle.position - neighbor.position
                distance = np.linalg.norm(relative_pos)
                if 0 < distance < self.influence_radius:
                    repulsive_force = self.k_rep * relative_pos / (distance ** 2 + 0.1)
                    total_force += repulsive_force
        
        # 4. 障碍物排斥力
        for obstacle in obstacles:
            relative_pos = vehicle.position - obstacle.position
            distance = np.linalg.norm(relative_pos) - obstacle.radius
            if 0 < distance < self.obstacle_radius:
                obstacle_force = self.k_obstacle * relative_pos / (distance ** 2 + 0.1)
                total_force += obstacle_force
        
        # 5. 阻尼力
        damping_force = -self.k_damping * vehicle.velocity
        total_force += damping_force
        
        return total_force

class PhysicsEngine:
    """物理引擎 - 模拟Gazebo物理特性"""
    
    def __init__(self, dt: float = 0.02):
        self.dt = dt
        self.gravity = 9.81
        
    def update_vehicle_dynamics(self, vehicle: VehicleState, control_force: np.ndarray):
        """更新车辆动力学 - 差分驱动模型"""
        
        # 限制控制力
        max_force = vehicle.mass * 5.0  # 最大加速度5m/s²
        force_magnitude = np.linalg.norm(control_force)
        if force_magnitude > max_force:
            control_force = control_force / force_magnitude * max_force
        
        # 计算加速度
        vehicle.acceleration = control_force / vehicle.mass
        
        # 更新速度
        vehicle.velocity += vehicle.acceleration * self.dt
        
        # 速度限制
        speed = np.linalg.norm(vehicle.velocity)
        if speed > vehicle.max_speed:
            vehicle.velocity = vehicle.velocity / speed * vehicle.max_speed
        
        # 更新位置
        vehicle.position += vehicle.velocity * self.dt
        
        # 更新航向角
        if speed > 0.1:
            vehicle.heading = np.arctan2(vehicle.velocity[1], vehicle.velocity[0])
        
        # 更新线速度
        vehicle.linear_velocity = speed
        
        # 简化的角速度更新
        if len(vehicle.velocity) >= 2 and speed > 0.1:
            # 基于速度变化计算角速度
            heading_change = np.arctan2(vehicle.acceleration[1], vehicle.acceleration[0]) - vehicle.heading
            vehicle.angular_velocity = heading_change / self.dt
            vehicle.angular_velocity = np.clip(vehicle.angular_velocity, 
                                             -vehicle.max_angular_speed, 
                                             vehicle.max_angular_speed)

class FormationSimulator:
    """多车编队仿真器主类"""
    
    def __init__(self, num_vehicles: int = 6):
        # 仿真参数
        self.num_vehicles = num_vehicles
        self.dt = 0.02  # 20ms - 贴合Gazebo默认步长
        self.simulation_time = 0.0
        self.is_running = False
        self.is_paused = False
        
        # 初始化组件
        self.controller = FormationController()
        self.physics = PhysicsEngine(self.dt)
        
        # 车辆和障碍物
        self.vehicles: List[VehicleState] = []
        self.obstacles: List[Obstacle] = []
        self.desired_positions = np.zeros((num_vehicles, 2))
        
        # 数据记录
        self.position_history = []
        self.error_history = []
        self.time_history = []
        
        # 初始化车辆
        self._initialize_vehicles()
        self._initialize_obstacles()
        
        # 可视化设置
        self.fig = None
        self.ax = None
        self.animation = None
        self.setup_visualization()
        
    def _initialize_vehicles(self):
        """初始化车辆"""
        self.vehicles = []
        
        # 初始位置 - 随机分布
        for i in range(self.num_vehicles):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(-2, 2)
            
            vehicle = VehicleState(
                id=i,
                position=np.array([x, y]),
                velocity=np.zeros(2),
                heading=0.0,
                angular_velocity=0.0,
                linear_velocity=0.0,
                acceleration=np.zeros(2),
                is_leader=(i == 0)  # 第一辆车为领导者
            )
            
            self.vehicles.append(vehicle)
    
    def _initialize_obstacles(self):
        """初始化障碍物"""
        self.obstacles = [
            Obstacle(position=np.array([8.0, 3.0]), radius=1.0),
            Obstacle(position=np.array([5.0, -2.0]), radius=0.8),
            Obstacle(position=np.array([12.0, 1.0]), radius=1.2),
        ]
    
    def setup_visualization(self):
        """设置可视化"""
        # 创建更大的窗口
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.ax.set_xlim(-5, 20)
        self.ax.set_ylim(-8, 12)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        self.ax.set_title('🚁 多车编队仿真系统 (实时可视化)', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('X 位置 (米)', fontsize=12)
        self.ax.set_ylabel('Y 位置 (米)', fontsize=12)
        
        # 设置背景色
        self.ax.set_facecolor('#f8f9fa')
        
        # 添加控制面板
        self._setup_control_panel()
        
    def _setup_control_panel(self):
        """设置控制面板"""
        # 为滑块和按钮留出空间
        plt.subplots_adjust(bottom=0.2, right=0.78, left=0.08)
        
        # 编队类型选择
        rax = plt.axes([0.8, 0.75, 0.18, 0.2])
        rax.set_title('编队类型', fontsize=10, fontweight='bold')
        self.formation_radio = RadioButtons(rax, 
            ('直线编队', 'V字编队', '菱形编队', '圆形编队'))
        # 映射中文到英文
        self.formation_map = {
            '直线编队': 'line',
            'V字编队': 'v_shape', 
            '菱形编队': 'diamond',
            '圆形编队': 'circle'
        }
        self.formation_radio.on_clicked(self._change_formation)
        
        # 编队尺度滑块
        scale_ax = plt.axes([0.15, 0.05, 0.5, 0.03])
        self.scale_slider = Slider(scale_ax, '编队尺度 (米)', 0.5, 5.0, 
                                  valinit=self.controller.formation_scale,
                                  valfmt='%.1f')
        self.scale_slider.on_changed(self._update_scale)
        
        # 速度滑块
        speed_ax = plt.axes([0.15, 0.1, 0.5, 0.03])
        self.speed_slider = Slider(speed_ax, '运动速度', 0.1, 3.0, 
                                  valinit=1.0, valfmt='%.1f')
        self.speed_slider.on_changed(self._update_speed)
        
        # 控制按钮
        start_ax = plt.axes([0.8, 0.55, 0.18, 0.06])
        self.start_button = Button(start_ax, '▶ 开始仿真', color='lightgreen')
        self.start_button.on_clicked(self._toggle_simulation)
        
        reset_ax = plt.axes([0.8, 0.47, 0.18, 0.06])
        self.reset_button = Button(reset_ax, '🔄 重置', color='lightcoral')
        self.reset_button.on_clicked(self._reset_simulation)
        
        # 状态信息显示区域
        self.info_ax = plt.axes([0.8, 0.2, 0.18, 0.25])
        self.info_ax.set_title('仿真状态', fontsize=10, fontweight='bold')
        self.info_ax.set_xticks([])
        self.info_ax.set_yticks([])
        self.info_ax.patch.set_facecolor('#e8f4fd')
        
    def _change_formation(self, label):
        """改变编队类型"""
        self.controller.formation_type = self.formation_map[label]
        
    def _update_scale(self, val):
        """更新编队尺度"""
        self.controller.formation_scale = self.scale_slider.val
        
    def _update_speed(self, val):
        """更新运动速度"""
        speed_factor = self.speed_slider.val
        for vehicle in self.vehicles:
            vehicle.max_speed = 2.0 * speed_factor
        
    def _toggle_simulation(self, event):
        """切换仿真状态"""
        if not self.is_running:
            self.start_simulation()
            self.start_button.label.set_text('⏸️ 暂停')
            self.start_button.color = 'lightyellow'
        else:
            self.is_paused = not self.is_paused
            if self.is_paused:
                self.start_button.label.set_text('▶️ 继续')
                self.start_button.color = 'lightgreen'
            else:
                self.start_button.label.set_text('⏸️ 暂停')
                self.start_button.color = 'lightyellow'
            
    def _reset_simulation(self, event):
        """重置仿真"""
        self.is_running = False
        self.is_paused = False
        self.simulation_time = 0.0
        self._initialize_vehicles()
        self.position_history.clear()
        self.error_history.clear()
        self.time_history.clear()
        
        # 重置按钮状态
        self.start_button.label.set_text('▶️ 开始仿真')
        self.start_button.color = 'lightgreen'
        
    def update_simulation(self):
        """更新仿真状态"""
        if self.is_paused:
            return
            
        # 更新编队目标位置
        self.desired_positions = self.controller.generate_formation_positions(self.num_vehicles)
        
        # 更新每辆车
        for i, vehicle in enumerate(self.vehicles):
            if not vehicle.is_active:
                continue
                
            # 计算控制力
            control_force = self.controller.calculate_formation_force(
                vehicle, self.desired_positions[i], self.vehicles, self.obstacles)
            
            # 更新物理状态
            self.physics.update_vehicle_dynamics(vehicle, control_force)
        
        # 记录数据
        self._record_data()
        
        # 更新时间
        self.simulation_time += self.dt
        
    def _record_data(self):
        """记录仿真数据"""
        positions = np.array([v.position for v in self.vehicles])
        self.position_history.append(positions.copy())
        
        # 计算编队误差
        error = np.mean([np.linalg.norm(self.vehicles[i].position - self.desired_positions[i]) 
                        for i in range(self.num_vehicles)])
        self.error_history.append(error)
        self.time_history.append(self.simulation_time)
        
    def animate(self, frame):
        """动画更新函数"""
        if self.is_running:
            self.update_simulation()
            
        self.ax.clear()
        self.ax.set_xlim(-5, 20)
        self.ax.set_ylim(-8, 12)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(f'多车编队仿真 - 时间: {self.simulation_time:.1f}s', fontsize=14)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        # 绘制轨迹 - 更清晰的轨迹线
        if len(self.position_history) > 2:
            for i in range(self.num_vehicles):
                trajectory = np.array([pos[i] for pos in self.position_history[-100:]])  # 最近100个点
                if self.vehicles[i].is_leader:
                    trail_color = '#FF8888'
                    trail_width = 2.5
                else:
                    colors = ['#6699FF', '#99DD66', '#FFDD44', '#BB77FF', '#44DDFF', '#FF9944']
                    trail_color = colors[i % len(colors)]
                    trail_width = 2.0
                
                # 渐变轨迹效果
                if len(trajectory) > 1:
                    for j in range(1, len(trajectory)):
                        alpha = j / len(trajectory) * 0.8  # 渐变透明度
                        self.ax.plot(trajectory[j-1:j+1, 0], trajectory[j-1:j+1, 1], 
                                   color=trail_color, alpha=alpha, linewidth=trail_width * alpha)
        
        # 绘制障碍物
        for obs in self.obstacles:
            circle = Circle(obs.position, obs.radius, color='red', alpha=0.6)
            self.ax.add_patch(circle)
            
        # 绘制目标位置
        target_circle = Circle(self.controller.target_position, 0.5, 
                             color='green', alpha=0.8)
        self.ax.add_patch(target_circle)
        self.ax.text(self.controller.target_position[0] + 0.6, 
                    self.controller.target_position[1], '目标', fontsize=10)
        
        # 绘制期望编队位置
        for i, pos in enumerate(self.desired_positions):
            circle = Circle(pos, 0.2, color='blue', alpha=0.3, linestyle='--')
            self.ax.add_patch(circle)
        
        # 绘制车辆 - 改进的可视化效果
        for i, vehicle in enumerate(self.vehicles):
            if not vehicle.is_active:
                continue
                
            # 车辆颜色方案
            if vehicle.is_leader:
                color = '#FF4444'  # 红色领导者
                edge_color = '#CC0000'
            else:
                colors = ['#4472C4', '#70AD47', '#FFC000', '#7030A0', '#00B0F0', '#FF6600']
                color = colors[i % len(colors)]
                edge_color = 'black'
            
            # 绘制车辆主体 (矩形)
            car_length = 0.6
            car_width = 0.4
            cos_h = np.cos(vehicle.heading)
            sin_h = np.sin(vehicle.heading)
            
            # 计算车辆四个角的位置
            corners = np.array([
                [-car_length/2, -car_width/2],
                [car_length/2, -car_width/2],
                [car_length/2, car_width/2],
                [-car_length/2, car_width/2]
            ])
            
            # 旋转和平移
            rotation_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
            rotated_corners = corners @ rotation_matrix.T + vehicle.position
            
            # 绘制车辆矩形
            vehicle_polygon = Polygon(rotated_corners, color=color, alpha=0.8, 
                                    edgecolor=edge_color, linewidth=2)
            self.ax.add_patch(vehicle_polygon)
            
            # 绘制车头方向指示器
            front_center = vehicle.position + np.array([cos_h * car_length/3, sin_h * car_length/3])
            front_circle = Circle(front_center, 0.08, color='white', alpha=0.9)
            self.ax.add_patch(front_circle)
            
            # 车辆ID标签 - 更明显
            label_pos = vehicle.position + np.array([0, car_width/2 + 0.3])
            self.ax.text(label_pos[0], label_pos[1], f'车辆{i}', 
                        fontsize=10, fontweight='bold', ha='center',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
            
            # 速度指示器 - 更明显的箭头
            if np.linalg.norm(vehicle.velocity) > 0.1:
                vel_scale = 1.5
                arrow_start = vehicle.position + np.array([cos_h * car_length/2, sin_h * car_length/2])
                self.ax.arrow(arrow_start[0], arrow_start[1],
                            vehicle.velocity[0] * vel_scale, 
                            vehicle.velocity[1] * vel_scale,
                            head_width=0.2, head_length=0.15, 
                            fc='yellow', ec='orange', alpha=0.8, linewidth=2)
            
            # 车辆状态显示
            speed = np.linalg.norm(vehicle.velocity)
            if speed > 0.05:
                status_pos = vehicle.position + np.array([0, -car_width/2 - 0.4])
                self.ax.text(status_pos[0], status_pos[1], f'{speed:.1f}m/s', 
                           fontsize=8, ha='center', color='blue', fontweight='bold')
        
        # 绘制编队连线
        if self.controller.formation_type in ['line', 'v_shape', 'diamond']:
            for i in range(len(self.vehicles) - 1):
                if self.vehicles[i].is_active and self.vehicles[i+1].is_active:
                    self.ax.plot([self.vehicles[i].position[0], self.vehicles[i+1].position[0]],
                               [self.vehicles[i].position[1], self.vehicles[i+1].position[1]],
                               'k--', alpha=0.3, linewidth=1)
        
        # 更新右侧状态信息面板
        self.info_ax.clear()
        self.info_ax.set_title('仿真状态', fontsize=10, fontweight='bold')
        self.info_ax.set_xticks([])
        self.info_ax.set_yticks([])
        self.info_ax.patch.set_facecolor('#e8f4fd')
        
        # 显示详细状态信息
        status_text = []
        status_text.append(f"⏱️ 时间: {self.simulation_time:.1f}s")
        status_text.append(f"🚗 车辆数: {self.num_vehicles}")
        
        # 编队类型显示
        formation_names = {
            'line': '直线编队',
            'v_shape': 'V字编队', 
            'diamond': '菱形编队',
            'circle': '圆形编队'
        }
        status_text.append(f"📐 编队: {formation_names.get(self.controller.formation_type, self.controller.formation_type)}")
        
        if len(self.error_history) > 0:
            status_text.append(f"📏 编队误差: {self.error_history[-1]:.2f}m")
        
        # 目标位置
        status_text.append(f"🎯 目标位置:")
        status_text.append(f"   X: {self.controller.target_position[0]:.1f}m")
        status_text.append(f"   Y: {self.controller.target_position[1]:.1f}m")
        
        # 平均速度
        avg_speed = np.mean([np.linalg.norm(v.velocity) for v in self.vehicles])
        status_text.append(f"⚡ 平均速度: {avg_speed:.2f}m/s")
        
        # 仿真状态
        if self.is_running:
            if self.is_paused:
                status_text.append("⏸️ 状态: 暂停")
            else:
                status_text.append("▶️ 状态: 运行中")
        else:
            status_text.append("⏹️ 状态: 停止")
        
        # 在信息面板中显示文本
        for i, text in enumerate(status_text):
            self.info_ax.text(0.05, 0.95 - i * 0.1, text, transform=self.info_ax.transAxes,
                            fontsize=9, verticalalignment='top')
        
        # 在主画面显示简化状态
        main_info = f"🚁 编队仿真系统 | 时间: {self.simulation_time:.1f}s | {formation_names.get(self.controller.formation_type, self.controller.formation_type)}"
        self.ax.text(0.02, 0.02, main_info, transform=self.ax.transAxes,
                    fontsize=10, bbox=dict(boxstyle='round,pad=0.5', 
                    facecolor='white', alpha=0.9, edgecolor='blue'))
        
    def start_simulation(self):
        """启动仿真"""
        self.is_running = True
        self.is_paused = False
        
        if self.animation is None:
            self.animation = animation.FuncAnimation(
                self.fig, self.animate, interval=int(self.dt * 1000), 
                blit=False, cache_frame_data=False)
        
    def show(self):
        """显示仿真界面"""
        # 添加鼠标点击事件处理
        def on_click(event):
            if event.inaxes == self.ax and event.dblclick:
                self.controller.target_position = np.array([event.xdata, event.ydata])
        
        self.fig.canvas.mpl_connect('button_press_event', on_click)
        
        plt.show()
        
    def save_data(self, filename: str = "formation_simulation_data.json"):
        """保存仿真数据"""
        data = {
            'simulation_time': self.simulation_time,
            'position_history': [pos.tolist() for pos in self.position_history],
            'error_history': self.error_history,
            'time_history': self.time_history,
            'formation_type': self.controller.formation_type,
            'num_vehicles': self.num_vehicles
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"仿真数据已保存到: {filename}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='多车编队仿真系统')
    parser.add_argument('-n', '--vehicles', type=int, default=6, 
                       help='车辆数量 (默认: 6)')
    parser.add_argument('-f', '--formation', type=str, default='line',
                       choices=['line', 'v_shape', 'diamond', 'circle'],
                       help='编队类型 (默认: line)')
    parser.add_argument('--auto-start', action='store_true',
                       help='自动开始仿真')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("🚁 多车编队仿真系统 (贴合Gazebo物理特性)")
    print("=" * 50)
    print(f"车辆数量: {args.vehicles}")
    print(f"编队类型: {args.formation}")
    print("\n操作说明:")
    print("- 双击设置目标位置")
    print("- 使用右侧控制面板调整参数")
    print("- 点击开始/暂停按钮控制仿真")
    print("- 点击重置按钮重新开始")
    print("=" * 50)
    
    # 创建仿真器
    simulator = FormationSimulator(num_vehicles=args.vehicles)
    simulator.controller.formation_type = args.formation
    
    if args.auto_start:
        simulator.start_simulation()
    
    # 启动仿真
    simulator.show()

if __name__ == "__main__":
    main() 