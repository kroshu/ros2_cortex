cmake_minimum_required(VERSION 3.5)
project(ros2_cortex)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(kroshu_ros2_core REQUIRED)
find_package(kuka_sunrise_interfaces REQUIRED)

find_library(kroshu_ros2_core_LIBRARIES kroshu_ros2_core /root/upstream_ws/build/kroshu_ros2_core)

include_directories(include /root/upstream_ws/src/kroshu_ros2_core/include)

add_library(cortex_client SHARED
  src/CortexMock.cpp
  src/CortexClient.cpp)

add_executable(simple_fod_printer
    src/SimpleFodPrinter.cpp
    src/CortexClientNode.cpp)
ament_target_dependencies(simple_fod_printer rclcpp rclcpp_lifecycle
  lifecycle_msgs kroshu_ros2_core kuka_sunrise_interfaces)
target_link_libraries(simple_fod_printer
  cortex_client ${kroshu_ros2_core_LIBRARIES})

add_executable(marker_publisher
    src/MarkerPublisher.cpp
    src/CortexClientNode.cpp)
ament_target_dependencies(marker_publisher rclcpp rclcpp_lifecycle visualization_msgs
  lifecycle_msgs kroshu_ros2_core kuka_sunrise_interfaces)
target_link_libraries(marker_publisher
  cortex_client ${kroshu_ros2_core_LIBRARIES})

add_executable(motion_tracker
    src/MotionTracker.cpp)
ament_target_dependencies(motion_tracker rclcpp rclcpp_lifecycle visualization_msgs
            lifecycle_msgs sensor_msgs std_msgs geometry_msgs kroshu_ros2_core)
target_link_libraries(motion_tracker
  ${kroshu_ros2_core_LIBRARIES})

target_include_directories(cortex_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${kroshu_ros2_core_INCLUDE_DIRS})

target_include_directories(simple_fod_printer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${kroshu_ros2_core_INCLUDE_DIRS})
target_include_directories(marker_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${kroshu_ros2_core_INCLUDE_DIRS})
target_include_directories(motion_tracker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${kroshu_ros2_core_INCLUDE_DIRS})
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

  file(GLOB_RECURSE rapidjson_main_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}/include/rapidjson"
    include/rapidjson/*.h)

  file(GLOB_RECURSE rapidjson_error_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}/include/rapidjson/error"
    include/rapidjson/error/*.h)

  file(GLOB_RECURSE rapidjson_internal_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}/include/rapidjson/internal"
    include/rapidjson/internal/*.h)

  file(GLOB_RECURSE rapidjson_msinttypes_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}/include/rapidjson/msinttypes"
    include/rapidjson/msinttypes/*.h)

  ament_copyright(--exclude ${rapidjson_main_headers} ${rapidjson_error_headers}
                  ${rapidjson_internal_headers} ${rapidjson_msinttypes_headers} Cortex.h)
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(--exclude ${rapidjson_main_headers} ${rapidjson_error_headers}
                  ${rapidjson_internal_headers} ${rapidjson_msinttypes_headers} Cortex.h)
  ament_xmllint()
endif()

ament_package()

######################
## Project analysis ##
######################

# Additional options for project analysis
set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
option(TEST_COVERAGE "Generate test coverage reports upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME};${PROJECT_SOURCE_DIR}\n")
  message(${SONARQUBE_PACKAGES_FILE})
endif()

if(TEST_COVERAGE)
    # Set build type to debug to avoid compiler optimization when checking coverage
    set(CMAKE_BUILD_TYPE Debug)
    # Include coverage report related functions
    list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake/CodeCoverage")
    include(CodeCoverage)
    # Append compiler flags to gcc for generating coverage notes
    append_coverage_compiler_flags()
    # Set coverage output dir to ${CMAKE_BINARY_DIR}/test_coverage
    set_coverage_output_dir()
endif()

if(TEST_COVERAGE)
    # Define a coverage target with some libs and executables to check
    #    NAME    Name of coverage target. Default: coverage. Only works with default currently.
    #    TARGETS Library and executable targets for which coverage reports are requested.
    # add_coverage_gcov(NAME coverage TARGETS mypkg_lib mypkg_node)
endif()
