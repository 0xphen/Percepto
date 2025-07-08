cmake_minimum_required(VERSION 3.16)
project(percepto_tests LANGUAGES CXX)

file(GLOB_RECURSE TEST_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp
)

add_executable(percepto_tests ${TEST_SOURCES})

target_include_directories(percepto_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/tests     
  ${CMAKE_SOURCE_DIR}/tests/unit
  ${CMAKE_SOURCE_DIR}/tests/integration
)

target_link_libraries(percepto_tests PRIVATE
    gtest_main
    percepto_sensor       
)

add_custom_command(TARGET percepto_tests PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
        $<TARGET_FILE_DIR:percepto_tests>/fixtures
)

gtest_discover_tests(percepto_tests
    WORKING_DIRECTORY $<TARGET_FILE_DIR:percepto_tests>
)