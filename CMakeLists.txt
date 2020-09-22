project(c++-pips)
cmake_minimum_required(VERSION 3.16.3)

# Tell CMake where it can find our custom .cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules/")

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set some useful compile flags
set(CMAKE_CXX_FLAGS "-march=native -Wall -ggdb -O2 -fPIC -fopenmp")
if(${CXX} MATCHES "g++")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
endif()

# ros setup stuff
INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

# Load required third-party libraries from system
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
# find_package(GFlags REQUIRED)
find_package(Threads REQUIRED)
find_package(Z3 REQUIRED)

# Set final locations for binaries
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Libraries
# SET(libs roslib roscpp rosbag boost_system pthread)
SET(libs roslib roscpp rosbag glog amrl_shared_lib boost_system lua5.1 gflags
    pthread)

# Load amrl_shared_lib from submodule
add_subdirectory(submodules/amrl_shared_lib/)
include_directories(submodules/amrl_shared_lib/)
include_directories(submodules/)

# Load Gflags from submodule
add_subdirectory(submodules/gflags)

# Load Niels Lohmann's JSON library from submodule
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(submodules/json)

# Build the core of C++ PIPS as a static library, which the executables can
# link to. This enables shorter compile times since these files only need to be
# built once.
set(CXX_PIPS_SRCS
    src/utils/nd_bool_array.cpp
    src/visitors/deepcopy_visitor.cpp
    src/visitors/deepcopy_visitor.hpp
    src/visitors/fillhole_visitor.cpp
    src/visitors/fillhole_visitor.hpp
    src/visitors/interp_visitor.cpp
    src/visitors/interp_visitor.hpp
    src/visitors/print_visitor.cpp
    src/visitors/print_visitor.hpp
    src/visitors/tosmtlib_visitor.cpp
    src/visitors/tosmtlib_visitor.hpp
    src/ast.cpp
    src/ast.hpp
    src/enumeration.cpp
    src/enumeration.hpp
    src/library_functions.cpp
    src/library_functions.hpp
    src/parsing.cpp
    src/parsing.hpp)
add_library(c++-pips-core ${CXX_PIPS_SRCS})
# target_link_libraries(c++-pips-core amrl_shared_lib)
target_link_libraries(c++-pips-core Eigen3::Eigen)
target_link_libraries(c++-pips-core nlohmann_json::nlohmann_json)
target_link_libraries(c++-pips-core ${Z3_LIBRARIES})
# target_link_libraries(c++-pips-core gflags::gflags)
target_link_libraries(c++-pips-core ${libs})

# Add an executable for ldips-l3
add_executable(ldips-l3 src/ldips-l3.cpp)
target_link_libraries(ldips-l3 c++-pips-core)
# target_link_libraries(ldips-l3 gflags::gflags)

rosbuild_add_executable(mpdm_ref src/policy/mpdm_ref.cpp)
target_link_libraries(mpdm_ref c++-pips-core)
target_link_libraries(mpdm_ref amrl_shared_lib)
