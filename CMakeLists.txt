# Works with 3.11 and tested through 3.19
cmake_minimum_required(VERSION 3.11...3.19)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# Project name and a few useful settings. Other commands can pick up the results
project(
  AMoD2
  VERSION 0.1
  DESCRIPTION "An agent-based modeling platform for mobility-on-demand simulations"
  LANGUAGES CXX)

########################################################################
# Find External Packages
########################################################################

# Define the path to cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required libraries
find_package(Boost 1.52.0 COMPONENTS filesystem system thread iostreams chrono date_time regex REQUIRED)

########################################################################
# Define Libraries and Executable
########################################################################

# The libraries
add_library(mod-abm-lib src/config.cpp src/demand_generator.cpp src/router.cpp src/vehicle.cpp src/utility.cpp)
target_link_libraries(mod-abm-lib yaml-cpp fmt::fmt ${Python3_INCLUDE_DIRS})
target_compile_features(mod-abm-lib PRIVATE cxx_std_17)

# The executable
add_executable(main src/main.cpp)
target_link_libraries(main mod-abm-lib yaml-cpp fmt::fmt)

target_compile_features(main PRIVATE cxx_std_17)

add_executable(test_gitignore src/test_gitignore.cpp)
target_link_libraries(test_gitignore mod-abm-lib yaml-cpp fmt::fmt)

target_compile_features(test_gitignore PRIVATE cxx_std_17)

# More linking for LibOSRM
include_directories(SYSTEM)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

########################################################################
# Fetch Other Third-Party Libraries
########################################################################

# Third-party libraries
include(FetchContent)
include(ExternalProject)

# Add fmt::fmt
FetchContent_Declare(
  fmtlib
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 5.3.0)
FetchContent_MakeAvailable(fmtlib)

# Add YAML
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.6.3)
FetchContent_MakeAvailable(yaml-cpp)

# Add Google test suite
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.8.0
)
FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  message(STATUS ${googletest_SOURCE_DIR})
  message(STATUS ${googletest_BINARY_DIR})
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()
