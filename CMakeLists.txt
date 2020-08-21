project(c++-pips)
cmake_minimum_required(VERSION 3.16.3)

# Tell CMake where it can find our custom .cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules/")

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set some useful compile flags
set(CMAKE_CXX_FLAGS "-march=native -Wall -ggdb -O2 -fopenmp")
if(${CXX} MATCHES "g++")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
endif()

# Set final locations for binaries
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Load required third-party libraries from system
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(Z3 REQUIRED)

# Load amrl_shared_lib from submodule
add_subdirectory(./submodules/amrl_shared_lib/)
include_directories(./submodules/amrl_shared_lib/)

# Load Gflags from submodule
add_subdirectory(./submodules/gflags)

# Load Niels Lohmann's JSON library from submodule
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(./submodules/json)

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
target_link_libraries(c++-pips-core amrl_shared_lib)
target_link_libraries(c++-pips-core Eigen3::Eigen)
target_link_libraries(c++-pips-core nlohmann_json::nlohmann_json)
target_link_libraries(c++-pips-core ${Z3_LIBRARIES})

# Add "main" runnable executable
add_executable(test src/main.cpp)
target_link_libraries(test c++-pips-core)
target_link_libraries(test gflags::gflags)

# Add executable for unit tests
add_executable(c++-pips-tests
  src/utils/nd_bool_array_tests.cpp
  src/visitors/deepcopy_visitor_tests.cpp
  src/library_functions_tests.cpp)
target_link_libraries(c++-pips-tests c++-pips-core)
target_link_libraries(c++-pips-tests ${GTEST_BOTH_LIBRARIES})
target_link_libraries(c++-pips-tests Threads::Threads)
gtest_add_tests(TARGET c++-pips-tests)
