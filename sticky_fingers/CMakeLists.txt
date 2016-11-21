cmake_minimum_required(VERSION 2.8.3)
project(sticky_fingers)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_service_files(
	FILES StickyControl.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp gazebo_ros
)

# Executable
add_executable(finger_control_dummy_node src/finger_control_dummy_node.cpp)

#Library
add_library(sticky_fingers src/sticky_fingers.cpp)
target_link_libraries(sticky_fingers ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(finger_control_dummy_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
