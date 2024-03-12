set(LIB srm_core)
message("Configuring core module...")

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC include
)

target_link_libraries(
  ${LIB}
  PUBLIC srm_common
  PUBLIC srm_video
  PUBLIC srm_viewer
  PUBLIC srm_nn
)
