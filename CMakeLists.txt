project(c++-pips)
cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules/")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g -fopenmp")
IF(${CXX} MATCHES "g++")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
ENDIF()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
ADD_SUBDIRECTORY(./submodules/amrl_shared_lib/)
INCLUDE_DIRECTORIES(./submodules/amrl_shared_lib/)
INCLUDE_DIRECTORIES(./lib/)

SET(libs amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

find_package(Z3 REQUIRED)
include_directories(${Z3_INCLUDE_DIRS})
set(CXX_PIPS_SRCS
    src/ast.cpp
    src/ast.hpp
    src/library_functions.cpp
    src/main.cpp)
add_executable(test ${CXX_PIPS_SRCS})
target_link_libraries(test ${libs})

include_directories(src)
set(CXX_PIPS_TEST_SRCS
    src/ast.cpp
    src/library_functions.cpp
    src/library_functions_tests.cpp)
add_executable(c++-pips-tests ${CXX_PIPS_TEST_SRCS})
target_link_libraries(c++-pips-tests ${GTEST_BOTH_LIBRARIES})
target_link_libraries(c++-pips-tests Threads::Threads)
target_link_libraries(c++-pips-tests ${libs})
gtest_add_tests(TARGET c++-pips-tests)
