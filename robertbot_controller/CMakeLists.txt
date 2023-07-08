cmake_minimum_required(VERSION 3.0.2)
project(robertbot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  hardware_interface
)

add_service_files(
  FILES
  AnglesConverter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    scripts/AngleConverter.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(AngleConverter_cpp src/AngleConverter.cpp)
add_dependencies(AngleConverter_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AngleConverter_cpp ${catkin_LIBRARIES})

add_executable(robertbot_interface_node src/robertbot_interface.cpp)
add_dependencies(robertbot_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robertbot_interface_node ${catkin_LIBRARIES})

