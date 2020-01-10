cmake_minimum_required(VERSION 2.8.3)
project(vant2load_Franklin_ZSE)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
simulator_msgs)

catkin_package()
include_directories(include)
INCLUDE_DIRECTORIES (/usr/include/eigen3)
include_directories( ${catkin_INCLUDE_DIRS})
include_directories($ENV{TILT_PROJECT}/source/Structure/control_strategies/)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

#add_library(vant2load_Franklin_ZSE src/main.cpp)
add_library(vant2load_Franklin_ZSE src/main.cpp include/feedforward.cpp)
target_link_libraries(vant2load_Franklin_ZSE ${catkin_LIBRARIES})
install(TARGETS
vant2load_Franklin_ZSE
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
