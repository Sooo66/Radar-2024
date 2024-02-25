set(LIB srm_common)
message("Configuring common module...")

find_package(OpenCV 4 REQUIRED QUIET)
find_package(Glog REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)

add_library(${LIB} INTERFACE)

target_include_directories(
  ${LIB}
  INTERFACE include
  INTERFACE ${OpenCV_INCLUDE_DIRS}
  INTERFACE ${GLOG_INCLUDE_DIRS}
  INTERFACE ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  ${LIB}
  INTERFACE ${OpenCV_LIBS}
  INTERFACE glog::glog
  INTERFACE ${CMAKE_THREAD_LIBS_INIT}
)
