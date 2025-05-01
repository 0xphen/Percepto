# tests/CMakeLists.txt

add_executable(percepto_tests
    TestVec3.cpp
)

target_link_libraries(percepto_tests
    gtest_main
    percepto_lib
)

include(GoogleTest)
gtest_discover_tests(percepto_tests)
