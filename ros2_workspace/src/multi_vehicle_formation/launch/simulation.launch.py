#!/usr/bin/env python3
"""
多车编队仿真启动文件
Multi-Vehicle Formation Simulation Launch File
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """生成启动描述"""
    
    # 包目录路径
    pkg_multi_vehicle_formation = FindPackageShare('multi_vehicle_formation')
    
    # 启动参数声明
    declare_num_vehicles = DeclareLaunchArgument(
        'num_vehicles',
        default_value='4',
        description='编队中的车辆数量'
    )
    
    declare_formation_type = DeclareLaunchArgument(
        'formation_type',
        default_value='line',
        description='编队类型: line, v_shape, diamond, circle'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='formation_world.sdf',
        description='Ignition Gazebo世界文件名'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )
    
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='是否启用详细输出'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动RViz可视化'
    )
    
    # 获取启动参数
    num_vehicles = LaunchConfiguration('num_vehicles')
    formation_type = LaunchConfiguration('formation_type')
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    rviz = LaunchConfiguration('rviz')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        pkg_multi_vehicle_formation,
        'config',
        'formation_params.yaml'
    ])
    
    vehicle_config_file = PathJoinSubstitution([
        pkg_multi_vehicle_formation,
        'config',
        'vehicle_params.yaml'
    ])
    
    # Ignition Gazebo启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    pkg_multi_vehicle_formation,
                    'worlds',
                    world_file
                ]),
                ' -r' if gui else ' -r -s'  # -r: run, -s: server only
            ],
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 编队控制器节点
    formation_controller = Node(
        package='multi_vehicle_formation',
        executable='formation_controller_node',
        name='formation_controller',
        parameters=[
            config_file,
            vehicle_config_file,
            {
                'use_sim_time': use_sim_time,
                'num_vehicles': num_vehicles,
                'formation_type': formation_type
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # 可视化节点
    formation_visualizer = Node(
        package='multi_vehicle_formation',
        executable='formation_visualizer_node',
        name='formation_visualizer',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'num_vehicles': num_vehicles
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # 车辆生成节点组
    vehicle_nodes = []
    for i in range(4):  # 默认4辆车，实际会根据num_vehicles参数动态调整
        vehicle_node = Node(
            package='multi_vehicle_formation',
            executable='vehicle_simulator_node',
            name=f'vehicle_{i}',
            namespace=f'vehicle_{i}',
            parameters=[
                vehicle_config_file,
                {
                    'use_sim_time': use_sim_time,
                    'vehicle_id': i,
                    'initial_x': 0.0 + i * 2.0,  # 初始x位置
                    'initial_y': 0.0,             # 初始y位置
                    'initial_yaw': 0.0            # 初始朝向
                }
            ],
            output='screen',
            condition=IfCondition(TextSubstitution(text=str(i < int(num_vehicles.perform(context)))))
        )
        vehicle_nodes.append(vehicle_node)
    
    # RViz可视化
    rviz_config_file = PathJoinSubstitution([
        pkg_multi_vehicle_formation,
        'config',
        'formation_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )
    
    # 静态变换发布器
    static_transforms = []
    for i in range(4):
        static_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_publisher_{i}',
            arguments=[
                '0', '0', '0',           # x, y, z
                '0', '0', '0', '1',      # qx, qy, qz, qw
                'map',                   # parent frame
                f'vehicle_{i}/base_link' # child frame
            ],
            condition=IfCondition(TextSubstitution(text=str(i < int(num_vehicles.perform(context)))))
        )
        static_transforms.append(static_transform)
    
    # ROS2-Ignition Gazebo桥接器
    bridge_config_file = PathJoinSubstitution([
        pkg_multi_vehicle_formation,
        'config',
        'gz_bridge.yaml'
    ])
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[bridge_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 机器人状态发布器
    robot_state_publishers = []
    for i in range(4):
        urdf_file = PathJoinSubstitution([
            pkg_multi_vehicle_formation,
            'models',
            'vehicle_robot',
            'model.urdf'
        ])
        
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{i}',
            namespace=f'vehicle_{i}',
            parameters=[
                {'robot_description': urdf_file},
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(TextSubstitution(text=str(i < int(num_vehicles.perform(context)))))
        )
        robot_state_publishers.append(robot_state_publisher)
    
    # 联合状态发布器
    joint_state_publishers = []
    for i in range(4):
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f'joint_state_publisher_{i}',
            namespace=f'vehicle_{i}',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(TextSubstitution(text=str(i < int(num_vehicles.perform(context)))))
        )
        joint_state_publishers.append(joint_state_publisher)
    
    # 数据记录节点 (可选)
    data_recorder = Node(
        package='multi_vehicle_formation',
        executable='data_recorder_node',
        name='data_recorder',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'record_frequency': 10.0,
                'output_file': '/tmp/formation_simulation_data.csv'
            }
        ],
        condition=IfCondition(LaunchConfiguration('record_data', default='false')),
        output='screen'
    )
    
    # 性能监控节点
    performance_monitor = Node(
        package='multi_vehicle_formation',
        executable='performance_monitor_node',
        name='performance_monitor',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'monitoring_frequency': 1.0
            }
        ],
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_num_vehicles)
    ld.add_action(declare_formation_type)
    ld.add_action(declare_world_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_verbose)
    ld.add_action(declare_rviz)
    
    # 添加隐式参数
    ld.add_action(DeclareLaunchArgument('record_data', default_value='false'))
    
    # 添加Gazebo
    ld.add_action(gazebo_launch)
    
    # 添加核心节点
    ld.add_action(formation_controller)
    ld.add_action(formation_visualizer)
    ld.add_action(performance_monitor)
    
    # 添加Gazebo桥接器
    ld.add_action(gz_bridge)
    
    # 添加车辆相关节点
    for vehicle_node in vehicle_nodes:
        ld.add_action(vehicle_node)
    
    for static_transform in static_transforms:
        ld.add_action(static_transform)
    
    for robot_state_publisher in robot_state_publishers:
        ld.add_action(robot_state_publisher)
    
    for joint_state_publisher in joint_state_publishers:
        ld.add_action(joint_state_publisher)
    
    # 添加可视化
    ld.add_action(rviz_node)
    
    # 添加可选节点
    ld.add_action(data_recorder)
    
    return ld 