cmake_minimum_required(VERSION 2.8.3)
project(cargo_ants_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Goal.msg
    TaskRequest.msg
    TaskRequestStatus.msg
    Task.msg
    TaskStatus.msg
    Path.msg
    PathStatus.msg
    TrajectoryPoint.msg
    Trajectory.msg
    VehicleState.msg
    MockupMapEntry.msg
    MockupMap.msg
)

generate_messages(
  DEPENDENCIES 
    std_msgs 
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES cargo_ants_msgs
  CATKIN_DEPENDS 
    std_msgs
    geometry_msgs
  #  DEPENDS system_lib
)
