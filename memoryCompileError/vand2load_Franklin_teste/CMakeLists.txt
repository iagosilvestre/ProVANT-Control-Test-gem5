cmake_minimum_required(VERSION 2.8.3)
project(vand2load_Franklin_teste)
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

add_library(vand2load_Franklin_teste src/main.cpp)
target_link_libraries(vand2load_Franklin_teste ${catkin_LIBRARIES})
install(TARGETS
vand2load_Franklin_teste
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
