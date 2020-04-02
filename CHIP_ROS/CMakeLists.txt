cmake_minimum_required(VERSION 2.8.3)
project(CHIP_ROS)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS  
  rospy 
  roscpp 
  std_msgs 
  sensor_msgs
)

## Uncomment if the package has a setup.py
catkin_python_setup()

#catkin_install_python(PROGRAMS src/scripts/chip_master.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS src/scripts/leg_load_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS src/scripts/rio_data_read_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS src/scripts/robot_COG_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS src/scripts/stabilizer_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS src/scripts/rio_write_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)