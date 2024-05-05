cmake_minimum_required(VERSION 3.8)
project(simple_move)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(exc_simple_move ./src/exc_simple_move.cpp)

ament_target_dependencies(exc_simple_move rclcpp moveit_core moveit_ros_planning_interface)

install(TARGETS exc_simple_move DESTINATION lib/${PROJECT_NAME})

ament_package()
