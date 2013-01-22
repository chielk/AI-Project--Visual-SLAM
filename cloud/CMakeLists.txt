project(datastructure)
find_package(OpenCV REQUIRED)
add_executable(datastructure datastructure.cpp)
target_link_libraries(datastructure ${OpenCV_LIBS})
