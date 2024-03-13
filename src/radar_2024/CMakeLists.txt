# 项目总配置
cmake_minimum_required(VERSION 3.12)
project(radar_2024)
set(EXECUTABLE_NAME radar_2024)

# 设置编译选项
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_IGNORE_PATH /opt/homebrew)
  set(CMAKE_PREFIX_PATH /usr/local)
  set(CMAKE_OSX_ARCHITECTURES x86_64)
  set(CMAKE_CXX_COMPILER clang++)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_COMPILER g++) # Ubuntu 20.04 or 22.04
    set(CMAKE_CXX_FLAGS -pthread)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_COMPILER g++-8) # Ubuntu 18.04
  endif()
endif()

# 设置优化参数
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  add_compile_options(-O0)
elseif(CMAKE_BUILD_TYPE STREQUAL Release)
  if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_compile_options(-O3 -march=x86-64)
  elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    add_compile_options(-O3 -march=native)
  endif()
endif()

# 设置输出位置
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

message("bin: {PROJECT_SOURCE_DIR}/bin")

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

catkin_package(CATKIN_DEPENDS
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

# 添加可执行文件
add_executable(lidar_node offlineCloudPoint.cpp)
add_executable(${EXECUTABLE_NAME} main.cpp)

target_include_directories(
  lidar_node
  PUBLIC ${catkin_INCLUDE_DIRS}
  PUBLIC ${PCL_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  lidar_node
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC ${PCL_LIBRARIES}
  PUBLIC ${EIGEN3_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(
  lidar_node
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC ${PCL_LIBRARIES}
  PUBLIC ${EIGEN3_LIBRARIES}
)

# 编译模块 注意顺序问题
add_subdirectory(modules/common)
add_subdirectory(modules/video)
add_subdirectory(modules/viewer)
add_subdirectory(modules/nn)
add_subdirectory(modules/lidar)
add_subdirectory(modules/core)


# 链接库主程序
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_link_libraries(
    ${EXECUTABLE_NAME} 
    PUBLIC srm_core
  )
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(
    ${EXECUTABLE_NAME} 
    PUBLIC -Wl,-no-as-needed srm_core
  )
endif()