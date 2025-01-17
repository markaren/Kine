
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(Eigen)

add_executable(crane3r main.cpp Crane3R.cpp)
target_link_libraries(crane3r PRIVATE threepp::threepp imgui::imgui Eigen3::Eigen kine)
