#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

# Get URDF file path
urdf_path = os.path.join(
    get_package_share_directory('alloy_chassis_mecanum'),
    'urdf',
    'alloy_chassis_mecanum.urdf')

print("URDF file path:", urdf_path)
print("File exists:", os.path.exists(urdf_path))

# Read URDF file
with open(urdf_path, 'r') as f:
    robot_description_content = f.read()

print("URDF content length:", len(robot_description_content))
print("First 500 characters:")
print(robot_description_content[:500])
print("\nLast 500 characters:")
print(robot_description_content[-500:])
