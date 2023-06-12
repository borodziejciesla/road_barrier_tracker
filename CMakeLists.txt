cmake_minimum_required (VERSION 3.22)

project("barrier_tracker" VERSION 0.0.1
  DESCRIPTION "C++ implementation of barrier tracker algorithm for autonmous vehicle"
  HOMEPAGE_URL "https://github.com/borodziejciesla/road_barrier_tracker")

set(TEST_PROJECT_NAME ${PROJECT_NAME}_tests)
set(EXAMPLE_PROJECT_NAME ${PROJECT_NAME}_example)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLE "Build example" ON)

###############################################
################## Library ####################
###############################################
find_package(Eigen3)

#add_subdirectory(components)

add_library(${PROJECT_NAME}
    core/src/barrier_tracker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    core/include
    # $<TARGET_PROPERTY:eigen,INTERFACE_INCLUDE_DIRECTORIES>
)

###############################################
################# Examples ####################
###############################################
# if(BUILD_EXAMPLE)
#     project(${EXAMPLE_PROJECT_NAME})
    
#     add_subdirectory(example)
# endif() # BUILD_EXAMPLE

###############################################
################ Unit Tests ###################
###############################################
# if(BUILD_TESTS)    
#     project(${TEST_PROJECT_NAME})

#     add_subdirectory(tests)    
#     enable_testing()
#     add_test(${TEST_PROJECT_NAME} ${TEST_PROJECT_NAME})    
# endif() #   BUILD_TESTS