project(c++-pips)
cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Z3 REQUIRED)
include_directories(${Z3_INCLUDE_DIRS})

set(CXX_PIPS_SRCS
    src/lang.cpp
    src/lang.hpp
    src/main.cpp)
add_executable(c++-pips ${CXX_PIPS_SRCS})
target_link_libraries(c++-pips ${Z3_LIBRARIES})