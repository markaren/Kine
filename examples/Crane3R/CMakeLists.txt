
add_executable(crane3r main.cpp Crane3R.cpp)
target_link_libraries(crane3r PUBLIC threepp::threepp imgui::imgui kine)
