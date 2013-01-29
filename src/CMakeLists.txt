cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Debug)
project(src)
#SET( CMAKE_CXX_COMPILER "/usr/bin/g++-4.6" )

# this lets you find the qibuild cmake framework
find_package(qibuild)
find_package(PCL 1.3 REQUIRED COMPONENTS visualization)

# Here, we create an executable named "getimages" from the cpp file.

set(_controller_srcs
  naocontroller.cpp
  inputsource.cpp
  inputsource.hpp
)

set(_salvo_srcs
  naostream.cpp
  inputsource.cpp
  inputsource.hpp
  cloud.cpp
  cloud.hpp
)
    
qi_create_bin(controller ${_controller_srcs})
qi_create_bin(salvo ${_salvo_srcs})


include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(salvo ${PCL_VISUALIZATION_LIBRARIES} ${PCL_COMMON_LIBRARIES})

# Here we say that our executable depends on
# - ALCOMMON (main naoqi lib)
# - ALVISION (for vision definitions)
# - OPENCV (display)
#
# It automatically links with the corresponding libraries and makes their headers
# available.
qi_use_lib(controller ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_calib3d )
qi_use_lib(salvo ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_calib3d OPENCV2_features2d )
