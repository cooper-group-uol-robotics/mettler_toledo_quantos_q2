cmake_minimum_required(VERSION 3.0.2)
project(mettler_toledo_quantos_q2_driver)


find_package(catkin REQUIRED COMPONENTS
  rospy
  mettler_toledo_quantos_q2_msgs
)


catkin_python_setup()


catkin_package(
	CATKIN_DEPENDS mettler_toledo_quantos_q2_msgs)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/quantos_q2_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


 install(FILES
    csv/response.csv
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

