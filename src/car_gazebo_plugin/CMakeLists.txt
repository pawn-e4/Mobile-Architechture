cmake_minimum_required(VERSION 3.5)
project(car_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(smartcar_msgs REQUIRED)

# Create the plugin library
add_library(car_gazebo_plugin SHARED src/car_gazebo_plugin.cpp)

ament_target_dependencies(car_gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "ackermann_msgs"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "smartcar_msgs"
)

# Export the library and include directories
ament_export_libraries(car_gazebo_plugin)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp gazebo_ros sensor_msgs tf2)

# Install the library
install(TARGETS car_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install launch files (if you add any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Finish package
ament_package()

