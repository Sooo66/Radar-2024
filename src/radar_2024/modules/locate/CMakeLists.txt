set(LIB srm_locate)
message("Configuring locate module...")

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC include
)

target_link_libraries(
  ${LIB}
  PUBLIC srm_common
  PUBLIC srm_nn
)
