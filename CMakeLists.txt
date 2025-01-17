cmake_minimum_required(VERSION 3.19)
project(kine VERSION 0.1.0)

option(KINE_BUILD_TESTS "Build tests" ON)
option(KINE_BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

include(FetchContent)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)


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

# ==============================================================================
# Exports and remaining installation
# ==============================================================================

configure_package_config_file(cmake/config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/kine-config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/kine
        NO_SET_AND_CHECK_MACRO)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/kine-config-version.cmake
        VERSION ${CMAKE_PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/kine-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/kine-config-version.cmake
        DESTINATION
        ${CMAKE_INSTALL_DATADIR}/kine)

install(TARGETS kine EXPORT kine-targets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT kine-targets
        NAMESPACE kine::
        DESTINATION ${CMAKE_INSTALL_DATADIR}/kine)