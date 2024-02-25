set(LIB srm_nn)
message("Configuring nn module...")

# 设置目录位置
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUDA_PATH /usr/local/cuda)  # Linux: CUDA 目录
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT) # Linux x86-64: TensorRT 目录
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
  endif()
endif()

# 查找软件包
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  find_package(CUDA REQUIRED)
  find_package(TensorRT REQUIRED)
endif()

# 添加源文件
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  aux_source_directory(src SRC)
  aux_source_directory(src/coreml SRC)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  aux_source_directory(src SRC)
endif()
add_library(${LIB} SHARED ${SRC})

# 设置库的路径
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_compile_options(${LIB} PRIVATE -x objective-c++ -fobjc-arc)
  set(CoreML_LIBS "-framework AppKit -framework CoreML -framework Foundation -framework Vision -framework CoreVideo")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  file(GLOB CUDA_LIBS ${CUDA_PATH}/lib64/libcu*.so)
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(${LIB} PRIVATE ${TensorRT_PATH}/lib)
    file(GLOB TensorRT_LIBS ${TensorRT_PATH}/lib/libnv*.so)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(${LIB} PRIVATE ${TensorRT_LIB_PATH})
    file(GLOB TensorRT_LIBS ${TensorRT_LIB_PATH}/libnv*.so)
  endif()
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_include_directories(
    ${LIB}
    PUBLIC include
  )
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_include_directories(
    ${LIB}
    PUBLIC ${TensorRT_INCLUDE_DIRS}
    PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
    PUBLIC include
  )
endif()

# 设置链接库
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_link_libraries(
    ${LIB}
    PRIVATE ${CoreML_LIBS}
    PUBLIC srm_common
  )
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(
    ${LIB}
    PRIVATE ${CUDA_LIBS}
    PRIVATE ${TensorRT_LIBS}
    PUBLIC srm_common
  )
endif()