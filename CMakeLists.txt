cmake_minimum_required(VERSION 3.5)
project(ros2_cortex)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(cortex_mock SHARED
  src/CortexMock.cpp
  src/CortexClient.cpp)
ament_target_dependencies(cortex_mock rclcpp rclcpp_lifecycle visualization_msgs lifecycle_msgs)

add_executable(simple_fod_printer
	src/SimpleFodPrinter.cpp)
ament_target_dependencies(simple_fod_printer rclcpp rclcpp_lifecycle visualization_msgs lifecycle_msgs)
target_link_libraries(simple_fod_printer
  cortex_mock)
  
add_executable(marker_publisher
	src/MarkerPublisher.cpp)
ament_target_dependencies(marker_publisher rclcpp rclcpp_lifecycle visualization_msgs lifecycle_msgs)
target_link_libraries(marker_publisher
  cortex_mock)
  
add_executable(motion_tracker
	src/MotionTracker.cpp)
ament_target_dependencies(motion_tracker rclcpp rclcpp_lifecycle visualization_msgs
			lifecycle_msgs sensor_msgs std_msgs geometry_msgs)

install(TARGETS simple_fod_printer marker_publisher motion_tracker
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)


  ament_copyright()
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
  ament_xmllint()
endif()

ament_package()
