cmake_minimum_required(VERSION 3.8.0)
project(imu_lpf)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
