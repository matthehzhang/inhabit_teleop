cmake_minimum_required(VERSION 3.16)
project(luke_unitree_programs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LUKE_UNITREE_LOCAL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../.local/unitree_robotics")

if(EXISTS "${LUKE_UNITREE_LOCAL_PREFIX}/lib/cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${LUKE_UNITREE_LOCAL_PREFIX}" "${LUKE_UNITREE_LOCAL_PREFIX}/lib/cmake")
endif()

find_package(unitree_sdk2 REQUIRED)

add_executable(g1_left_wrist_serial g1_left_wrist_serial.cpp)
target_link_libraries(g1_left_wrist_serial PRIVATE unitree_sdk2)
