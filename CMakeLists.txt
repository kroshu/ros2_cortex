cmake_minimum_required(VERSION 3.5)
project(ros2_cortex)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
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
    #   NAME    Name of coverage target. Default: coverage. Only works with default currently.
    #   TARGETS Library and executable targets for which coverage reports are requested.
    # add_coverage_gcov(NAME coverage TARGETS mypkg_lib mypkg_node)
endif()

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
    #   NAME    Name of coverage target. Default: coverage. Only works with default currently.
    #   TARGETS Library and executable targets for which coverage reports are requested.
    # add_coverage_gcov(NAME coverage TARGETS mypkg_lib mypkg_node)
endif()
