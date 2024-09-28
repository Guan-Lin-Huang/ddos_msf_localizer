cmake_minimum_required(VERSION 3.0.2)
project(ddos_msf_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(Eigen3 QUIET)
find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  autoware_config_msgs
  autoware_msgs
  autoware_can_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ublox_msgs
  novatel_gps_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################
# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Msf_state.msg
  Position.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES msf_localizer
 CATKIN_DEPENDS
 autoware_config_msgs
 autoware_msgs
 autoware_can_msgs
 geometry_msgs
 nav_msgs
 roscpp
 rospy
 sensor_msgs
 std_msgs
 tf
 ublox_msgs
 novatel_gps_msgs
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
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ddos_msf_localizer src/node.cpp src/msf_localizer.cpp src/mec_transformation.cpp)
add_dependencies(ddos_msf_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ddos_msf_localizer ${catkin_LIBRARIES})

add_executable(imu_calibration src/imu_calibration.cpp)
target_link_libraries(imu_calibration ${catkin_LIBRARIES})

add_executable(data_collect src/data_collect.cpp)
target_link_libraries(data_collect ${catkin_LIBRARIES})


#############
## Install ##
#############



## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS #${PROJECT_NAME}
  ddos_msf_localizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


#############
## Testing ##
#############