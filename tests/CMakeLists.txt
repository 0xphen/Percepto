# tests/CMakeLists.txt

add_executable(percepto_tests
    TestVec3.cpp
    TestRay.cpp
    TestRayBuilder.cpp
    TestSphere.cpp
    TestMathUtils
)

target_link_libraries(percepto_tests
    gtest_main
    percepto_lib
)

# Automatically discover all TEST() cases
include(GoogleTest)
gtest_discover_tests(percepto_tests)
