cmake_minimum_required(VERSION 3.8)
project(alloy_chassis_mecanum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  launch
  meshes
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/fake_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_odom_publisher
)

install(PROGRAMS
  scripts/serial_odom_receiver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME serial_odom_receiver
)

ament_package()
