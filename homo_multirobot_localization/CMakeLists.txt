cmake_minimum_required(VERSION 3.8)
project(homo_multirobot_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mocap_state_adapter src/mocap_state_adapter.cpp)
target_compile_features(mocap_state_adapter PUBLIC cxx_std_17)
target_include_directories(mocap_state_adapter PUBLIC include)
ament_target_dependencies(mocap_state_adapter rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS mocap_state_adapter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
