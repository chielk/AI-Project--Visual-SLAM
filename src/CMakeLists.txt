cmake_minimum_required(VERSION 2.8)
project(src)

# this lets you find the qibuild cmake framework
find_package(qibuild)

# Here, we create an executable named "getimages" from the cpp file.

set(_srcs
  naocontroller.cpp
  inputsource.cpp
  inputsource.hpp
)
    
qi_create_bin(naoimage ${_srcs})

# Here we say that our executable depends on
# - ALCOMMON (main naoqi lib)
# - ALVISION (for vision definitions)
# - OPENCV (display)
#
# It automatically links with the corresponding libraries and make their headers
# available.
qi_use_lib(naoimage ALCOMMON ALVISION OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_calib3d)
