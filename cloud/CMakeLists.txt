project(datastructure)
find_package(OpenCV REQUIRED)
add_executable(cloud cloud.cpp)
target_link_libraries(cloud ${OpenCV_LIBS})
