#ifndef GETIMAGES_HPP
#define GETIMAGES_HPP

// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <math.h>

using namespace boost;
using namespace AL;
using namespace std;

class naoCamera {
    string clientName;
    ALVideoDeviceProxy *camProxy;
    ALMotionProxy *motProxy;
    string robotIP;

    void unsubscribe(const string);
    void subscribe(string, int);
    void detectBriskFeatures();
    void setProxies();


public:
    naoCamera(const string&);
    void cameraCalibration();
    void showImages();
    cv::Mat cameraMatrix, distCoeffs;
    void recordDataSet();
};

#endif // GETIMAGES_HPP
