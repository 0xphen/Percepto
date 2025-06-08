cmake_minimum_required(VERSION 3.14)
project(Percepto VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

add_subdirectory(external/csv-parser)
add_subdirectory(src)
target_link_libraries(percepto_lib
    PRIVATE
      csv    # “csv” is the target that csv-parser’s CMake created
)

enable_testing()
add_subdirectory(tests)
