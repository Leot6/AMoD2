# Works with 3.11 and tested through 3.19
cmake_minimum_required(VERSION 3.11...3.19)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# Project name and a few useful settings. Other commands can pick up the results
project(
  AMoD2
  DESCRIPTION "An agent-based modeling platform for mobility-on-demand simulations"
  LANGUAGES CXX)

include_directories(SYSTEM src)

########################################################################
# Find External Packages
########################################################################

# Define the path to cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Define the path to gurobi library (Mac OS)
set(GUROBI_DIR /Library/gurobi911/mac64)    # if Gurobi directory/version is different, manually change this line and "gurobi91" in line 38
include_directories(${GUROBI_DIR}/include)  # path to: "gurobi_c++.h"
link_directories(${GUROBI_DIR}/lib)         # path to: the Gurobi C++ library "libgurobi_c++.a" and the Gurobi C library "libgurobi91.dylib"

# Find all required libraries
find_package(Boost 1.52.0 COMPONENTS filesystem system thread iostreams chrono date_time regex REQUIRED)

########################################################################
# Define Libraries and Executable
########################################################################

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
# At least cmake 3.14.5 is needed to support CMake command "FetchContent_MakeAvailable".

# Add YAML
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.6.3)
FetchContent_MakeAvailable(yaml-cpp)
