cmake_minimum_required(VERSION 3.8)
project(multi_vehicle_formation)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 设置包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 创建自定义消息和服务
set(msg_files
  "msg/VehicleState.msg"
  "msg/FormationCommand.msg"
  "msg/FormationStatus.msg"
)

set(srv_files
  "srv/ChangeFormation.srv"
  "srv/SetFormationParams.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

# 编队控制核心库
add_library(formation_core
  src/formation_algorithm.cpp
  src/vehicle_controller.cpp
  src/path_planner.cpp
)

target_include_directories(formation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(formation_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# 编队控制节点
add_executable(formation_controller_node
  src/formation_controller_node.cpp
)

target_link_libraries(formation_controller_node formation_core)

ament_target_dependencies(formation_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# 可视化节点
add_executable(formation_visualizer_node
  src/formation_visualizer_node.cpp
)

ament_target_dependencies(formation_visualizer_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# 确保消息生成在节点编译之前
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(formation_controller_node "${cpp_typesupport_target}")
target_link_libraries(formation_visualizer_node "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS
  formation_controller_node
  formation_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件
install(TARGETS formation_core
  EXPORT formation_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装世界文件
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 安装模型文件
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# 导出库和依赖
ament_export_targets(formation_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # 代码风格检查
  ament_lint_auto_find_test_dependencies()
  
  # 单元测试
  ament_add_gtest(test_formation_algorithm
    test/test_formation_algorithm.cpp
  )
  target_link_libraries(test_formation_algorithm formation_core)
  
  ament_add_gtest(test_vehicle_controller
    test/test_vehicle_controller.cpp
  )
  target_link_libraries(test_vehicle_controller formation_core)
endif()

ament_package() 