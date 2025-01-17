cmake_minimum_required(VERSION 3.29)
project(kine)

option(KINE_BUILD_TESTS "Build tests" ON)
option(KINE_BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")


add_subdirectory(src)


if (KINE_BUILD_EXAMPLES)

    add_subdirectory(examples)

endif ()

if (KINE_BUILD_TESTS)

    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.4.0
    )
    FetchContent_MakeAvailable(Catch2)

    enable_testing()
    add_subdirectory(tests)
endif ()
