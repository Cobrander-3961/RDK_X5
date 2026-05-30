cmake_minimum_required(VERSION 3.22)
project(elite_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Python 运动规划节点
install(
  PROGRAMS
    scripts/arm_motion_planner.py
    scripts/arm_fixed_grasp_test.py
    scripts/arm_reach_test.py
    scripts/arm_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
