project(c++-pips)
cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")
IF(${CXX} MATCHES "g++")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
ENDIF()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Z3 REQUIRED)
include_directories(${Z3_INCLUDE_DIRS})
set(CXX_PIPS_SRCS
    src/ast.cpp
    src/ast.hpp
    src/main.cpp)
add_executable(test ${CXX_PIPS_SRCS})

