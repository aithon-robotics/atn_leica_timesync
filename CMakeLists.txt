cmake_minimum_required(VERSION 3.16)
project(atn_leica_timesync)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include/leica_timesync
    LIBRARIES leica_timesync
    CATKIN_DEPENDS roscpp geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/leica_timesync
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/leica_timesync.cc src/LowPassFilter.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## Add dependencies to the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
