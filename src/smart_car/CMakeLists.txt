cmake_minimum_required(VERSION 3.8)
project(smart_car)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smartcar_msgs REQUIRED)





# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install urdf files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(PROGRAMS
  scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)



# Package
ament_package()

