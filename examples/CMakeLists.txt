
file(COPY "data/" DESTINATION "${CMAKE_BINARY_DIR}/bin/data/")

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
set(THREEPP_WITH_AUDIO OFF)
set(THREEPP_WITH_SVG OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 63dd86ba3c02a1e8b9ff3fd396657cfe22206b81
)
FetchContent_MakeAvailable(threepp)

add_subdirectory(Crane3R)
add_subdirectory(external)
