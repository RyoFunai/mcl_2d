cmake_minimum_required(VERSION 3.8)
project(mcl_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(mcl_2d
  src/mcl_2d.cpp
  src/util.cpp
  src/mcl_2d_node.cpp
  src/msg_converter.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  ../launch
  ../config
)

