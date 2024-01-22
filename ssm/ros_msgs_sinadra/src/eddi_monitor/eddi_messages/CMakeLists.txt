cmake_minimum_required(VERSION 3.0.2)
project(eddi_messages)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation

)

add_message_files(
  DIRECTORY msg
  FILES
  SafeMLOutput.msg
  BayesianNetworkOutput.msg
  OutcomeOutput.msg
  ConSertOutput.msg
  NodeOutput.msg
  Guarantee.msg

)

generate_messages(
  DEPENDENCIES

)

catkin_package(
  CATKIN_DEPENDS
  message_runtime

)
