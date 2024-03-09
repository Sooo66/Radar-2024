set(LIB srm_lidar)
message("Configuring lidar module...")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  image_transport
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# catkin_package(CATKIN_DEPENDS
#   roscpp
#   rospy
#   roslib
#   std_msgs
#   sensor_msgs
#   cv_bridge
#   message_generation
#   message_runtime
#   image_transport
# )

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC include
  PUBLIC ${catkin_INCLUDE_DIRS}
  PUBLIC ${PCL_LIBRARY_DIRS}
)

target_link_libraries(
  ${LIB}
  PUBLIC ${PCL_LIBRARIES}  
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC srm_common
)
