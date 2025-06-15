# tests/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(percepto_tests LANGUAGES CXX)

# 1) Gather all .cpp under tests/unit and tests/integration
file(GLOB_RECURSE TEST_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp
)

add_executable(percepto_tests ${TEST_SOURCES})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# 2) Include paths: your public include + unit/integration helpers
target_include_directories(percepto_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests     
    ${CMAKE_SOURCE_DIR}/tests/unit
    ${CMAKE_SOURCE_DIR}/tests/integration
)


# 3) Link GoogleTest and your library
target_link_libraries(percepto_tests PRIVATE
    gtest_main
    percepto_lib
)

# 4) Copy test DATA (CSV, headers, etc.) into the build tree
add_custom_command(TARGET percepto_tests PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/fixtures
        $<TARGET_FILE_DIR:percepto_tests>/fixtures
)

# 5) Register the test; set its working directory so it sees “data/triangles.csv”
include(GoogleTest)
gtest_discover_tests(percepto_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
