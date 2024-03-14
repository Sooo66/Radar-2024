set(LIB srm_video)
message("Configuring video module...")

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(MVS_PATH /Library/MVS_SDK) # MacOS: 海康相机驱动目录
  if(NOT EXISTS ${MVS_PATH})
    message(FATAL_ERROR "MVS not found.")
  endif()
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(MVS_PATH /opt/MVS) # Linux: 海康相机驱动目录
  if(NOT EXISTS ${MVS_PATH})
    message(FATAL_ERROR "MVS not found.")
  endif()
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(MVS_INCLUDE_DIRS ${MVS_PATH}/Includes)
  file(GLOB MVS_LIBS ${MVS_PATH}/lib/*.dylib)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(MVS_INCLUDE_DIRS ${MVS_PATH}/include)
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
  endif()
endif()

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PRIVATE ${MVS_INCLUDE_DIRS}
  PUBLIC include
)

target_link_libraries(
  ${LIB} 
  PRIVATE ${MVS_LIBS}
  PUBLIC srm_common
)
