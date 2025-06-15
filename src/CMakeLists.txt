# src/CMakeLists.txt
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS *.cpp)

add_library(percepto_lib ${SOURCES})
target_include_directories(percepto_lib
    PUBLIC
      ${PROJECT_SOURCE_DIR}/include  # PROJECT_SOURCE_DIR is the root “percepto/”
)