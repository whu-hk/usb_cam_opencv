cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(usb_cam_opencv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp 
  std_msgs  
  sensor_msgs 
  image_transport
  cv_bridge)
find_package(OpenCV REQUIRED)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 # INCLUDE_DIRS 
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#将cpp声明为库
add_library(usb_cam_opencv src/usb_cam_opencv.cpp)
target_link_libraries(usb_cam_opencv ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(usb_cam_opencv ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} usb_cam_opencv_generate_messages_cpp)

#生成node可执行文件
add_executable(usb_cam_opencv_node src/nodes/usb_cam_opencv_node.cpp)
target_link_libraries(usb_cam_opencv_node usb_cam_opencv ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(usb_cam_opencv_node usb_cam_opencv_generate_messages_cpp) 
