cmake_minimum_required(VERSION 3.10)

project(Percepto VERSION 0.1.0 LANGUAGES C CXX)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# Include CTest (optional testing support)
include(CTest)

# Add headers from this folder
include_directories(src)

# Grab all .cpp files under src/
file(GLOB_RECURSE SOURCES src/*.cpp)

# Build the executable
add_executable(percepto ${SOURCES})

# Optional: if you want to enforce C++20 for this target
# (Change the target name to match exactly)
set_property(TARGET percepto PROPERTY CXX_STANDARD 20)
