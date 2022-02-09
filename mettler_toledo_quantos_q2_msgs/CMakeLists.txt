cmake_minimum_required(VERSION 3.0.2)
project(mettler_toledo_quantos_q2_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   QuantosCommand.msg
   QuantosResponse.msg
   QuantosSample.msg
 )

 generate_messages(
   DEPENDENCIES
std_msgs
 )


catkin_package(
	CATKIN_DEPENDS message_runtime)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
