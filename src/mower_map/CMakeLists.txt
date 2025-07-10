cmake_minimum_required(VERSION 3.0.2)
project(mower_map)

## === Compile options ===
# add_compile_options(-std=c++11)  # Déjà activé implicitement avec ROS Noetic

## === Find dependencies ===
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_cv
  rosbag
  xbot_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    MapArea.msg
    MapAreas.msg
)

add_service_files(
  FILES
    AddMowingAreaSrv.srv
    AppendMapSrv.srv
    ConvertToNavigationAreaSrv.srv
    DeleteMowingAreaSrv.srv
    GetDockingPointSrv.srv
    GetMowingAreaSrv.srv
    SetDockingPointSrv.srv
    ClearNavPointSrv.srv
    SetNavPointSrv.srv
    ClearMapSrv.srv
    PersistNextGlobalPlanSrv.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin-specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    grid_map_core
    grid_map_ros
    grid_map_filters
    grid_map_cv
    xbot_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mower_map_service
  src/mower_map_service.cpp
)

add_dependencies(mower_map_service
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(mower_map_service
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

## Install executable
install(TARGETS mower_map_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install message and service definitions
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  FILES_MATCHING PATTERN "*.msg"
)

install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
  FILES_MATCHING PATTERN "*.srv"
)

## Optionnel : installer les fichiers launch plus tard
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

#############
## Testing ##
#############

# catkin_add_gtest(...)  # À activer plus tard si besoin

#############################################
## === TODO: SLAM / cartographie dynamique ===
## Ce package permet de gérer les zones de tonte, le docking et les points de navigation.
## Des extensions futures pourraient inclure :
##  - Fusion SLAM (RTAB-Map, GMapping)
##  - Mise à jour en temps réel de la carte (grid_map)
##  - Export dynamique vers Nav2 / NavFn / GlobalPlanner
#############################################

