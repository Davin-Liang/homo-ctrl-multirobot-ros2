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
find_package(osqp_vendor REQUIRED)

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

# ---- 6D 鐢垫満鎰熺煡妯″瀷鎺у埗鍣紙鎵ц鍣ㄤ竴闃舵粸鍚庡骞匡紝doc/6d_motor_model_design.md锛?---
# 娉ㄦ剰: 瀹炶溅 ARM 鍐呭瓨涓嶈冻鏃跺彲娉ㄩ噴鎺夋湰 target锛堝悓涓嬫柟 6D 绯诲垪锛?
# add_executable(formation_control_node_6d_motor
#   src/main_6d_motor.cpp
#   src/formation_control_node_6d_motor.cpp
# )

# target_include_directories(formation_control_node_6d_motor PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_6d_motor
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_6d_motor
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_6d_motor
#   DESTINATION lib/${PROJECT_NAME}
# )

# ---- 4D Artstein-HPC 鎺у埗鍣紙杈撳叆鏃跺欢绛変环鍙樻崲锛宒oc/artstein_reduction.md锛?---
add_executable(formation_control_node_4d_artstein
  src/main_4d_artstein.cpp
  src/formation_control_node_4d_artstein.cpp
)

target_include_directories(formation_control_node_4d_artstein PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_4d_artstein
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_4d_artstein
  Eigen3::Eigen
)

install(TARGETS formation_control_node_4d_artstein
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 4D Artstein-MPC 对照控制器（同一预测层 + 4D linear MPC）---------------
add_executable(formation_control_node_4d_artstein_mpc
  src/main_4d_artstein_mpc.cpp
  src/formation_control_node_4d_artstein_mpc.cpp
)

target_include_directories(formation_control_node_4d_artstein_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_4d_artstein_mpc
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  osqp_vendor
)

target_link_libraries(formation_control_node_4d_artstein_mpc
  Eigen3::Eigen
  osqp::osqp
)

install(TARGETS formation_control_node_4d_artstein_mpc
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 6D Artstein Disc 控制器（方向 A: 平移/偏航预测 + 6D Disc HPC）--------
add_executable(formation_control_node_6d_artstein_disc
  src/main_6d_artstein_disc.cpp
  src/formation_control_node_6d_artstein_disc.cpp
)

target_include_directories(formation_control_node_6d_artstein_disc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_6d_artstein_disc
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_6d_artstein_disc
  Eigen3::Eigen
)

install(TARGETS formation_control_node_6d_artstein_disc
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 8D Pade 姝诲尯澧炲箍鎺у埗鍣紙doc/pade_deadtime_full.md锛?--------------------
# add_executable(formation_control_node_8d_motor
#   src/main_8d_motor.cpp
#   src/formation_control_node_8d_motor.cpp
# )

# target_include_directories(formation_control_node_8d_motor PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_8d_motor
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_8d_motor
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_8d_motor
#   DESTINATION lib/${PROJECT_NAME}
# )

# ---- 4D 杩炵画杈圭晫鎶曞奖鎺у埗鍣?----------------------------------------------------
# add_executable(formation_control_node_4d_cont
#   src/main_4d_cont.cpp
#   src/formation_control_node_4d_cont.cpp
# )

# target_include_directories(formation_control_node_4d_cont PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_4d_cont
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_4d_cont
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_4d_cont
#   DESTINATION lib/${PROJECT_NAME}
# )

# ---- 浠ヤ笅鐩爣鍦ㄥ皬杞︿笂鍐呭瓨涓嶅缂栬瘧锛屾殏鏃惰烦杩?---- 鍙湪 WSL/寮烘満鍣ㄤ笂鍙栨秷娉ㄩ噴
# ---- 6D 杩愬姩瀛︽ā鍨嬫帶鍒跺櫒 (Layer 3) -------------------------------------------
# add_executable(formation_control_node_6d
#   src/main_6d.cpp
#   src/formation_control_node_6d.cpp
# )

# target_include_directories(formation_control_node_6d PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_6d
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_6d
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_6d
#   DESTINATION lib/${PROJECT_NAME}
# )

# ---- 6D 鏂逛綅瑙掔害鏉熺紪闃熸帶鍒跺櫒 ---------------------------------------------------
# add_executable(formation_control_node_6d_bearing
#   src/main_6d_bearing.cpp
#   src/formation_control_node_6d_bearing.cpp
# )

# target_include_directories(formation_control_node_6d_bearing PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_6d_bearing
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_6d_bearing
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_6d_bearing
#   DESTINATION lib/${PROJECT_NAME}
# )

# ---- 6D 绂绘暎澶氳竟褰㈢紪闃熸帶鍒跺櫒 ---------------------------------------------------
# add_executable(formation_control_node_6d_disc
#   src/main_6d_disc.cpp
#   src/formation_control_node_6d_disc.cpp
# )

# target_include_directories(formation_control_node_6d_disc PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_6d_disc
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_6d_disc
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_6d_disc
#   DESTINATION lib/${PROJECT_NAME}
# )

## ---- 6D+OA 杩愬姩瀛︽ā鍨嬫帶鍒跺櫒 (甯﹂伩闅? -----------------------------------------
# add_executable(formation_control_node_6d_oa
#   src/main_6d_oa.cpp
#   src/formation_control_node_6d_oa.cpp
#   src/obstacle_avoider.cpp
# )

# target_include_directories(formation_control_node_6d_oa PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# ament_target_dependencies(formation_control_node_6d_oa
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# target_link_libraries(formation_control_node_6d_oa
#   Eigen3::Eigen
# )

# install(TARGETS formation_control_node_6d_oa
#   DESTINATION lib/${PROJECT_NAME}
# )

## ---- MPC 6D 杩愬姩瀛︽ā鍨嬫帶鍒跺櫒 (瀵圭収缁? --------------------------------------
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
  scripts/record_velocity_diagnostics.py
  scripts/formation_control_py.py
  scripts/virtual_leader_circle.py
  scripts/measure_motor_latency.py
  scripts/measure_cross_machine_delay.py
  scripts/sim_motor_delay.py
  scripts/sim_4d_hpc_artstein_compare.py
  scripts/sim_4d_artstein_mpc_compare.py
  scripts/sim_6d_disc_artstein_compare.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_mpc_controller_4d_artstein
    test/test_mpc_controller_4d_artstein.cpp
  )
  target_include_directories(test_mpc_controller_4d_artstein PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(test_mpc_controller_4d_artstein
    Eigen3::Eigen
    osqp::osqp
  )
  add_test(NAME test_mpc_controller_4d_artstein
    COMMAND test_mpc_controller_4d_artstein)

  add_executable(test_formation_radial_safety
    test/test_formation_radial_safety.cpp
  )
  target_include_directories(test_formation_radial_safety PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(test_formation_radial_safety
    Eigen3::Eigen
  )
  add_test(NAME test_formation_radial_safety
    COMMAND test_formation_radial_safety)
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

