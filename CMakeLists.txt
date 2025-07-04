cmake_minimum_required(VERSION 3.14)
project(Percepto VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
FetchContent_Declare(
  csv_parser
  GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
  GIT_TAG        2.2.1
)

FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.3.2
)

FetchContent_MakeAvailable(
  googletest
  tomlplusplus
  spdlog
  csv_parser
  CLI11
)

FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG v1.8.3
)

set(BENCHMARK_ENABLE_TESTING OFF)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF)

FetchContent_MakeAvailable(benchmark)

file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(percepto_lib ${LIB_SOURCES})

target_include_directories(percepto_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(percepto_lib
  PUBLIC
    tomlplusplus::tomlplusplus
    spdlog::spdlog
    csv 
    CLI11::CLI11
    benchmark
)

add_executable(percepto
  src/main.cpp
)

target_link_libraries(percepto
  PRIVATE
    percepto_lib
)

enable_testing()
add_subdirectory(tests)
add_subdirectory(benches)

