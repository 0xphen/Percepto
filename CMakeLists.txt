cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(Percepto VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

FetchContent_Declare(
  googletest 
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_Declare(
  tomlplusplus
  GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
  GIT_TAG        v3.4.0
)
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.13.0
)
# FetchContent_Declare(
#   csv_parser
#   GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
# )
FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.3.2
)
FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG v1.8.3
)

add_subdirectory(external/csv-parser)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(
  googletest
  tomlplusplus
  spdlog
  CLI11
  benchmark
)

include(GoogleTest)

set(PERCEPTO_GLOBAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

function(add_percepto_common_settings target_name)
    set_target_properties(${target_name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ON
    )
endfunction()

add_library(percepto_core STATIC
  src/core/config_loader.cpp
  src/math/math_utils.cpp
)
target_include_directories(percepto_core PUBLIC
  ${PERCEPTO_GLOBAL_INCLUDE_DIR}
)
target_link_libraries(percepto_core PRIVATE
  tomlplusplus::tomlplusplus
  spdlog::spdlog
)
add_percepto_common_settings(percepto_core)

add_library(percepto_scene STATIC
  src/core/scene.cpp                
  src/math/intersection/moller_trumbore.cpp
  src/io/csv_parser.cpp
)
target_include_directories(percepto_scene PUBLIC
  ${PERCEPTO_GLOBAL_INCLUDE_DIR}
)
target_link_libraries(percepto_scene PRIVATE
  percepto_core
  csv
  spdlog::spdlog
)
add_percepto_common_settings(percepto_scene)

add_library(percepto_lidar STATIC
  src/lidar/emitter.cpp
  src/lidar/simulator.cpp
)
target_include_directories(percepto_lidar PUBLIC
  ${PERCEPTO_GLOBAL_INCLUDE_DIR}
)
target_link_libraries(percepto_lidar PRIVATE
  percepto_core
  percepto_scene
  spdlog::spdlog
)
add_percepto_common_settings(percepto_lidar)

add_executable(percepto
  src/main.cpp
)
target_include_directories(percepto PRIVATE
  ${PERCEPTO_GLOBAL_INCLUDE_DIR}
)
target_link_libraries(percepto PRIVATE
  percepto_lidar
  CLI11::CLI11 
  tomlplusplus::tomlplusplus
  spdlog::spdlog
)
add_percepto_common_settings(percepto)

enable_testing()
add_subdirectory(tests)
add_subdirectory(benches)