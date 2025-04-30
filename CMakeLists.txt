cmake_minimum_required(VERSION 3.10)

project(percepto)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

include_directories(src)

# Recursively collect all .cpp files
file(GLOB_RECURSE SOURCES
    src/*.cpp
)

add_executable(percepto ${SOURCES})