cmake_minimum_required(VERSION 3.8)
project(smartcar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_lint_auto REQUIRED)  # Must be BEFORE ament_package()

# Only run lint checks if testing
if(BUILD_TESTING)
  # skip copyright and cpplint checks if desired
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate ROS 2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Finalize package
ament_package()

