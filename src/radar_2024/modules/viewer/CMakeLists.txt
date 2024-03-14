set(LIB srm_viewer)
message("Configuring viewer module...")

aux_source_directory(src SRC)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC include
)

target_link_libraries(
  ${LIB} 
  PUBLIC srm_common
)
