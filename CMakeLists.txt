cmake_minimum_required(VERSION 3.0.2)
project(franka_app_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package( Franka REQUIRED )
link_directories(${Franka_LIBRARY_DIRS})
add_definitions(${Franka_DEFINITIONS})

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED )
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES franka_app_cpp
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(examples_common
  include/franka_app/examples_common.h
  src/generate_cartesian_motion/examples_common.cpp
 )

target_link_libraries(examples_common ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(examples_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${Franka_EXPORTED_TARGETS} ${Eigen_EXPORTED_TARGETS})
# ${${PROJECT_NAME}_EXPORTED_TARGETS}

add_executable(franka_test src/franka_test.cpp)
target_link_libraries(franka_test ${catkin_LIBRARIES} ${Franka_LIBRARIES})

add_executable(franka_test_impedance src/impedance_control_example.cpp)
target_link_libraries(franka_test_impedance ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(controller_sub src/franka_controller_sub.cpp)
target_link_libraries(controller_sub ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${OpenCV_LIBS})

add_executable(generate_cart_pose_motion src/generate_cartesian_motion/generate_cartesian_pose_motion.cpp)
target_link_libraries(generate_cart_pose_motion examples_common ${catkin_LIBRARIES} ${Franka_LIBRARIES})




## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
