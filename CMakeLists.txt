cmake_minimum_required(VERSION 2.8.3)
project(vio_data_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
)


catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/gener_alldata.cpp src/param.cpp src/utilities.cpp src/imu.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node_txt
    src/gener_alldata_txt.cpp src/param.cpp src/utilities.cpp src/imu.cpp)
