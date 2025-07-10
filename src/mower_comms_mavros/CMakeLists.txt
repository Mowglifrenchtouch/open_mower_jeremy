cmake_minimum_required(VERSION 3.0.2)
project(mower_comms_mavros)

## Compile avec C++14
add_compile_options(-std=c++14)

## Rechercher les dépendances
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  mavros_msgs
  cv_bridge
)

## OpenCV (optionnel si tu n'en as pas besoin partout)
find_package(OpenCV REQUIRED)

###################################
## Déclaration de ce package ROS ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mower_comms_mavros
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf mavros_msgs cv_bridge
  DEPENDS OpenCV
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node src/mower_comms_MAVROS.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config interfaces services
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
                 PATTERN "*.yaml"
                 PATTERN "*.json"
)
