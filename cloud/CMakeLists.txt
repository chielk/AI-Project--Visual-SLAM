project(datastructure)
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud main.cpp cloud.cpp cloud.hpp)

target_link_libraries(cloud ${OpenCV_LIBS})
target_link_libraries (cloud ${PCL_LIBRARIES})
