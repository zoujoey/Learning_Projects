cmake_minimum_required(VERSION 3.0.2)
project(ODK_Filter)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(

)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Noisy_Data_Generator
  src/ex1_KFilter.cpp)

add_executable(ODVK_Filter
  src/main_Kfilter.cpp
  src/ODK_Filter.cpp
)
target_link_libraries(Noisy_Data_Generator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

target_link_libraries(ODVK_Filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})