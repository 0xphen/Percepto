# src/CMakeLists.txt

# Gather .cpp files (e.g Vec3 is header-only so no Vec3.cpp)
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS *.cpp)

# Create a library target
add_library(percepto_lib ${SOURCES})

# Include headers from src/
target_include_directories(percepto_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
