cmake_minimum_required(VERSION 3.0.9)
project(nuturtlebot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
   FILES
   SensorData.msg
   WheelCommands.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_runtime)
