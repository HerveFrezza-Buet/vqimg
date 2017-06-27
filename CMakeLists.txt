cmake_minimum_required(VERSION 2.8.3)
project(vqimg)

# PkgConfig stuff


find_package(PkgConfig)
pkg_check_modules(VQ2 vq2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  image_transport 
  cv_bridge
  dynamic_reconfigure
  message_generation)



generate_dynamic_reconfigure_options(
  cfg/gngt.cfg
  cfg/hue.cfg
  )


add_message_files(
  FILES
  component_center.msg
  component_centers.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  image_transport
  cv_bridge
  message_runtime
  )


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


## Declare a cpp executable


add_executable              (gngt_node src/gngt_node.cpp         )
target_link_libraries       (gngt_node        ${catkin_LIBRARIES})
target_link_libraries       (gngt_node        ${VQ2_LIBRARIES}   )
target_compile_options      (gngt_node PUBLIC -O3                )
target_compile_options      (gngt_node PUBLIC ${VQ2_CFLAGS_OTHER})
target_include_directories  (gngt_node PUBLIC ${VQ2_INCLUDE_DIRS})
add_dependencies            (gngt_node vqimg_gencfg              )

add_executable              (hue_filter_node src/hue_filter_node.cpp)
target_link_libraries       (hue_filter_node ${catkin_LIBRARIES}    )
target_compile_options      (hue_filter_node PUBLIC -O3 -std=c++11  )
add_dependencies            (hue_filter_node vqimg_gencfg           )
