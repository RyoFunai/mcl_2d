cmake_minimum_required(VERSION 3.8)
project(mcl_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)  # 追加


include_directories(include)

ament_auto_add_executable(mcl_2d
  src/mcl_2d.cpp
  src/util.cpp
  src/mcl_2d_node.cpp
  src/msg_converter.cpp
)

target_link_libraries(mcl_2d yaml-cpp ${OpenCV_LIBS})  # 追加


ament_auto_package(INSTALL_TO_SHARE
  ../launch
  ../config
)

