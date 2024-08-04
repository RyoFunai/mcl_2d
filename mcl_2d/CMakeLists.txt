cmake_minimum_required(VERSION 3.8)
project(mcl_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

include_directories(include)

ament_auto_add_library(mcl_2d_core SHARED
  src/mcl_2d.cpp
  src/particle_sampler.cpp
  src/util.cpp
  src/msg_converter.cpp
)

target_link_libraries(mcl_2d_core yaml-cpp ${OpenCV_LIBS})

ament_auto_add_executable(mcl_2d
  # src/mcl_2d.cpp
  # src/util.cpp
  src/mcl_2d_node.cpp
  # src/msg_converter.cpp
)

target_link_libraries(mcl_2d mcl_2d_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mcl_2d_test test/mcl_2d_test.cpp)
  target_include_directories(mcl_2d_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(mcl_2d_test mcl_2d_core)
  ament_target_dependencies(mcl_2d_test rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros)
endif()

ament_auto_package(INSTALL_TO_SHARE
  ../launch
  ../config
)