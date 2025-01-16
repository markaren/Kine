cmake_minimum_required(VERSION 3.29)
project(kine)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(src)

include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

enable_testing()
add_subdirectory(tests)
