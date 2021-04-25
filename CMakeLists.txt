# Works with 3.11 and tested through 3.19
cmake_minimum_required(VERSION 3.11...3.19)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# Project name and a few useful settings. Other commands can pick up the results
project(
  AMoD2
  DESCRIPTION "An agent-based modeling platform for mobility-on-demand simulations"
  LANGUAGES CXX)

########################################################################
# Find External Packages
########################################################################
if (CMAKE_HOST_APPLE)
    set(MACOS 1)
elseif (CMAKE_HOST_UNIX)
    set(LINUX 1)
endif ()

# Define the path to cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Define the path to gurobi library
# if Gurobi directory/version is different, manually change the directory to Gurobi in set(GUROBI_DIR ...) (line 30/33)
# and term "gurobi91" in target_link_libraries(mod-abm-lib ... gurobi91) (line 53).
if (MACOS)
    MESSAGE(STATUS "operation system is MacOs (${CMAKE_SYSTEM})")
    set(GUROBI_DIR /Library/gurobi911/mac64)
elseif (LINUX)
    MESSAGE(STATUS "operation system is Linux (${CMAKE_SYSTEM})")
    set(GUROBI_DIR /Library/gurobi911/linux64)
endif ()
    # path to: "gurobi_c++.h"
include_directories(${GUROBI_DIR}/include)
    # path to: the Gurobi C++ library "libgurobi_c++.a"
    # and the Gurobi C library "libgurobi91.dylib(Mac OS) / libgurobi91.so(Linux)"
link_directories(${GUROBI_DIR}/lib)

# Find all required libraries
find_package(Boost 1.52.0 COMPONENTS filesystem system thread iostreams chrono date_time regex REQUIRED)

########################################################################
# Define Libraries and Executable
########################################################################
include_directories(SYSTEM src)

# The libraries
add_library(mod-abm-lib src/simulator/config.cpp src/simulator/demand_generator.cpp src/simulator/router.cpp
        src/simulator/vehicle.cpp src/utility/utility_functions.cpp src/dispatcher/scheduling.cpp
        src/dispatcher/ilp_assign.cpp)
target_link_libraries(mod-abm-lib yaml-cpp fmt::fmt gurobi_c++ gurobi91)
target_compile_features(mod-abm-lib PRIVATE cxx_std_17)

# The executable
add_executable(main src/main.cpp)
target_link_libraries(main mod-abm-lib)
target_compile_features(main PRIVATE cxx_std_17)

add_executable(test_gitignore src/test_gitignore.cpp)
target_link_libraries(test_gitignore mod-abm-lib)
target_compile_features(test_gitignore PRIVATE cxx_std_17)

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
    # At least cmake 3.11.0 (sometimes the error message says 3.14.5) is needed to support CMake command "FetchContent_MakeAvailable".

# Add YAML
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.6.3)
FetchContent_MakeAvailable(yaml-cpp)
