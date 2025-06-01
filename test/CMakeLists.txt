cmake_minimum_required(VERSION 3.10)
project(percepto_tests)

# Automatically collect all test source files under this directory
file(GLOB_RECURSE TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

# Define the test executable
add_executable(percepto_tests ${TEST_SOURCES})

# Include directories:
#  - project headers (percepto_lib)
#  - tests root (e.g. for TestHelpers.h)
target_include_directories(percepto_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against GoogleTest and your library
target_link_libraries(percepto_tests
    PRIVATE
        gtest_main
        percepto_lib
)

# Discover and register all TEST() cases automatically
include(GoogleTest)
gtest_discover_tests(percepto_tests)