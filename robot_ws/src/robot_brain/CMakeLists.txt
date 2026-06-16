cmake_minimum_required(VERSION 3.22)
project(robot_brain)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/brain_pipeline.py
    scripts/doubao_client.py
    scripts/stereo_detect.py
    scripts/yolo_stereo_detect.py
    scripts/voice_chat.py
    scripts/auto_grasp.py
    scripts/auto_grasp_full.py
    scripts/patrol.py
    scripts/yolo_test.py
    scripts/camera_utils.py
    scripts/navigator.py
    scripts/grasper.py
    scripts/detector.py
    scripts/integrated_patrol.py
    scripts/voice_commander.py
    scripts/fruit_analyzer.py
    scripts/cloud_uploader.py
    scripts/orchard_patrol.py
    scripts/stereo_calibrate.py
    scripts/hand_eye_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES resource/robot_brain DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
