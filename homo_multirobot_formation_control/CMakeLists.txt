cmake_minimum_required(VERSION 3.8)
project(homo_multirobot_formation_control)

# Default to C++17 (std::clamp, etc.)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(osqp_vendor QUIET)

add_executable(formation_control_node
  src/main.cpp
  src/formation_control_node.cpp
)

target_include_directories(formation_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node
  Eigen3::Eigen
)

install(TARGETS formation_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 4D 连续边界投影控制器 ----------------------------------------------------
add_executable(formation_control_node_4d_cont
  src/main_4d_cont.cpp
  src/formation_control_node_4d_cont.cpp
)

target_include_directories(formation_control_node_4d_cont PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_4d_cont
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_4d_cont
  Eigen3::Eigen
)

install(TARGETS formation_control_node_4d_cont
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 以下目标在小车上内存不够编译，暂时跳过 ---- 可在 WSL/强机器上取消注释
# ---- 6D 运动学模型控制器 (Layer 3) -------------------------------------------
add_executable(formation_control_node_6d
  src/main_6d.cpp
  src/formation_control_node_6d.cpp
)

target_include_directories(formation_control_node_6d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_6d
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_6d
  Eigen3::Eigen
)

install(TARGETS formation_control_node_6d
  DESTINATION lib/${PROJECT_NAME}
)

## ---- 6D+OA 运动学模型控制器 (带避障) -----------------------------------------
add_executable(formation_control_node_6d_oa
  src/main_6d_oa.cpp
  src/formation_control_node_6d_oa.cpp
  src/obstacle_avoider.cpp
)

target_include_directories(formation_control_node_6d_oa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_6d_oa
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_6d_oa
  Eigen3::Eigen
)

install(TARGETS formation_control_node_6d_oa
  DESTINATION lib/${PROJECT_NAME}
)

## ---- MPC 6D 运动学模型控制器 (对照组) --------------------------------------
#add_executable(formation_control_node_mpc_6d
#  src/main_mpc_6d.cpp
#  src/formation_control_node_mpc_6d.cpp
#)
#
#target_include_directories(formation_control_node_mpc_6d PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#)
#
#ament_target_dependencies(formation_control_node_mpc_6d
#  rclcpp
#  geometry_msgs
#  nav_msgs
#  tf2
#  tf2_ros
#  tf2_geometry_msgs
#  osqp_vendor
#)
#
#target_link_libraries(formation_control_node_mpc_6d
#  Eigen3::Eigen
#  osqp::osqp
#)
#
#install(TARGETS formation_control_node_mpc_6d
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/leader_control.py
  scripts/leader_circle.py
  scripts/leader_eight.py
  scripts/record_trajectory.py
  scripts/formation_control_py.py
  scripts/virtual_leader_circle.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
