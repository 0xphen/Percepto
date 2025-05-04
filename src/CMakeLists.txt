# src/CMakeLists.txt

# Gather all .cpp implementation files under src/
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS *.cpp)

# Create core library target
add_library(percepto_lib ${SOURCES})

# Add public headers directory so other targets can access them via #include <percepto/Vec3.h>
target_include_directories(percepto_lib
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
