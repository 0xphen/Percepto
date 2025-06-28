# benchmark/CMakeLists.txt

# Find all .cpp files in this folder
file(GLOB BENCHMARK_FILES CONFIGURE_DEPENDS *.cpp)

# Loop over each file and build a separate executable
foreach(file ${BENCHMARK_FILES})
  get_filename_component(fname ${file} NAME_WE)

  add_executable(${fname} ${file})

  target_link_libraries(${fname}
    PRIVATE
      benchmark       # Linked because of FetchContent in root
      percepto_lib    # Your core simulation library
  )

  target_include_directories(${fname}
    PRIVATE
      ${CMAKE_SOURCE_DIR}/include
  )
endforeach()
