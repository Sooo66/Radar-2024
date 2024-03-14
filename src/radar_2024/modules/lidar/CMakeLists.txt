set(LIB srm_lidar)
message("Configuring lidar module...")

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})
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

target_include_directories(
  ${LIB}
  PUBLIC include
  PUBLIC ${catkin_INCLUDE_DIRS}
  PUBLIC ${PCL_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  ${LIB}
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC ${PCL_LIBRARIES}
  PUBLIC ${EIGEN3_LIBRARIES}
)
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(
  ${LIB}
  PUBLIC srm_common
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC ${PCL_LIBRARIES}
  PUBLIC ${EIGEN3_LIBRARIES}
)
