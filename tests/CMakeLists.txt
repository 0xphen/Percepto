cmake_minimum_required(VERSION 3.10)
project(percepto_tests)

file(GLOB_RECURSE TEST_SOURCES
    ${CMAKE_SOURCE_DIR}/tests/*.cpp
)

add_executable(percepto_tests ${TEST_SOURCES})

target_include_directories(percepto_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/tests
)

target_link_libraries(percepto_tests
    PRIVATE
        gtest_main
        percepto_lib
)

include(GoogleTest)
gtest_discover_tests(percepto_tests)
