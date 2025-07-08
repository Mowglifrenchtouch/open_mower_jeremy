cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mower_comms_mavros)

find_package(catkin REQUIRED COMPONENTS
  mower_msgs
  sensor_msgs
  roscpp
  xesc_msgs
  xbot_msgs
  robot_localization
)

find_package(Boost REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

catkin_package(
  DEPENDS GeographicLib
)

# === Includes ===
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/mower_comms_mavros/services
  ${PROJECT_SOURCE_DIR}/mower_comms_mavros/src
)

# === Source Files ===
set(SOURCE_FILES
  src/mower_comms_MAVROS.cpp
  src/MowerUIServiceInterface.cpp
  src/EmergencyServiceInterfaceMAVROS.cpp
  src/DiffDriveServiceInterfaceMAVROS.cpp
  src/MowerServiceInterfaceMAVROS.cpp
  src/ImuServiceInterfaceMAVROS.cpp
  src/PowerServiceInterfaceMAVROS.cpp
  src/GpsServiceInterfaceMAVROS.cpp
)

# === Header Files (optional, for IDEs) ===
set(HEADER_FILES
  src/EmergencyServiceInterfaceMAVROS.h
  src/DiffDriveServiceInterfaceMAVROS.h
  src/MowerServiceInterfaceMAVROS.h
  src/ImuServiceInterfaceMAVROS.h
  src/PowerServiceInterfaceMAVROS.h
  src/GpsServiceInterfaceMAVROS.h
  src/MowerUIServiceInterface.h
)
set_source_files_properties(${HEADER_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
# === Executable ===
add_executable(mower_comms_mavros ${SOURCE_FILES} )

add_service_interface(
  MowerUIServiceInterface
  ${CMAKE_CURRENT_SOURCE_DIR}/src/MowerUIService.interface
)
# === Interface auto-generation from JSON ===
set(SERVICE_FILES
  diff_drive_service
  emergency_service
  gps_service
  imu_service
  mower_service
  power_service
  mower_ui_service
)




foreach(service IN LISTS SERVICE_FILES)
  string(REPLACE "_service" "" class ${service})  # Simple transformation if needed
  target_add_service_interface(mower_comms_mavros ${class}ServiceInterface ${CMAKE_CURRENT_SOURCE_DIR}/services/${service}.json)
endforeach()

# === Dependencies ===
add_dependencies(mower_comms_mavros ${catkin_EXPORTED_TARGETS})
target_link_libraries(mower_comms_mavros
  PUBLIC
    ${catkin_LIBRARIES}
    ${GeographicLib_LIBRARIES}
)
