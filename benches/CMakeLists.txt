cmake_minimum_required(VERSION 3.16)
project(percepto_benchmarks LANGUAGES CXX)

set(GOOGLE_BENCHMARK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/moller_trumbore_benchmarks.cpp
)

add_executable(percepto_micro_benchmarks ${GOOGLE_BENCHMARK_SOURCES})

target_link_libraries(percepto_micro_benchmarks
    PRIVATE
        benchmark 
        percepto_sensor
        tomlplusplus::tomlplusplus
        spdlog::spdlog
)

target_include_directories(percepto_micro_benchmarks PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include 
  ${benchmark_SOURCE_DIR}/include
)

add_executable(run_scan_full_duration ${CMAKE_CURRENT_SOURCE_DIR}/run_scan_full_duration.cpp)

target_link_libraries(run_scan_full_duration
    PRIVATE
        percepto_sensor
        tomlplusplus::tomlplusplus
        spdlog::spdlog
)
target_include_directories(run_scan_full_duration PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)