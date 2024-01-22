cmake_minimum_required(VERSION 3.0.2)
project(kios_sinadra_scenario_2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs

  message_generation
)

generate_messages(
   DEPENDENCIES
  std_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kios_sinadra_scenario_2
   CATKIN_DEPENDS rospy std_msgs  message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
