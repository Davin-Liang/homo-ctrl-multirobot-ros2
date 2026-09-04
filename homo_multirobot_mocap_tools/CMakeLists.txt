cmake_minimum_required(VERSION 3.8)
project(homo_multirobot_mocap_tools)

find_package(ament_cmake REQUIRED)
find_package(VRPN REQUIRED)

add_library(vrpn_test_server_core src/vrpn_test_server_core.cpp)
target_compile_features(vrpn_test_server_core PUBLIC cxx_std_17)
target_include_directories(vrpn_test_server_core PUBLIC include)

add_executable(vrpn_test_server src/vrpn_test_server.cpp)
target_compile_features(vrpn_test_server PRIVATE cxx_std_17)
target_include_directories(vrpn_test_server PRIVATE ${VRPN_INCLUDE_DIRS})
target_link_libraries(vrpn_test_server PRIVATE vrpn_test_server_core ${VRPN_LIBRARIES})

install(TARGETS vrpn_test_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vrpn_test_server test/test_vrpn_test_server.cpp)
  target_link_libraries(test_vrpn_test_server vrpn_test_server_core)
endif()

ament_package()
