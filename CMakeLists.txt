cmake_minimum_required(VERSION 3.14)
project(Percepto VERSION 0.1.0 LANGUAGES C CXX)

cmake_policy(SET CMP0135 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GoogleTest Setup
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

# Add subdirectories (order matters!)
add_subdirectory(src)
enable_testing()
add_subdirectory(test)
