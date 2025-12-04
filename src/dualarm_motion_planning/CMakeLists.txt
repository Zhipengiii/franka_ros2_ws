cmake_minimum_required(VERSION 3.8)
project(dualarm_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED) 

# 2. 生成自定义消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskAction.msg"
  "msg/TaskActionList.msg"
  "msg/ObjectGoal.msg"
  "srv/GiveGoal.srv"
  "srv/TrajPara.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# 3. 包含头文件目录
include_directories(include)

# 4. 编译共享库
add_library(${PROJECT_NAME}_lib
  src/path_planner.cpp
  src/collision_detection.cpp
  src/find_point_on_line.cpp
  src/path_tracking_planning.cpp
  src/dualarm_coordinator.cpp
)

# 将当前源码目录的绝对路径定义为宏 PACKAGE_SOURCE_DIR 传入代码
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 5. 为库添加依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  ament_index_cpp 
)

# 6. 链接自定义消息接口
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# 7. 编译可执行文件
add_executable(control src/control.cpp)
add_executable(coord_node src/coord_node.cpp)
add_executable(task_distribution src/task_distribution.cpp)

# 为可执行文件添加依赖
ament_target_dependencies(control 
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  control_msgs
  trajectory_msgs
  sensor_msgs
  ament_index_cpp 
)

ament_target_dependencies(coord_node 
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  control_msgs
  trajectory_msgs
  sensor_msgs
  ament_index_cpp 
)

ament_target_dependencies(task_distribution 
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  control_msgs
  trajectory_msgs
  sensor_msgs
  ament_index_cpp 
)

# 链接生成的库
target_link_libraries(control ${PROJECT_NAME}_lib)
target_link_libraries(coord_node ${PROJECT_NAME}_lib)
target_link_libraries(task_distribution ${PROJECT_NAME}_lib)
# 传入源码路径宏
target_compile_definitions(control PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(coord_node PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 8. 安装规则
install(TARGETS 
  ${PROJECT_NAME}_lib 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 9. 安装可执行文件
install(TARGETS control DESTINATION lib/${PROJECT_NAME})
install(TARGETS coord_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS task_distribution DESTINATION lib/${PROJECT_NAME})

# 安装 Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本 (以便使用 ros2 run dualarm_motion_planning task_sequence.py 运行)
install(PROGRAMS scripts/task_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()