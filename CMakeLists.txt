cmake_minimum_required(VERSION 2.8)
project(diablo_simulation)

find_package(gazebo REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  cmake_modules
)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        /usr/include/eigen3
        )

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
)

add_library(generic_motor_plugin SHARED src/motor_controller.cpp)
target_link_libraries(generic_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(low_ctrl_plugin SHARED src/low_level_controller.cpp)
target_link_libraries(low_ctrl_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
