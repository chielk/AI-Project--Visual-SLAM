#ifndef GETIMAGES_HPP
#define GETIMAGES_HPP

#define LEFT    65361
#define UP      65362
#define RIGHT   65363
#define DOWN    65364
#define ESC     27


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
#include <boost/thread.hpp>

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
    void sweep(bool *);
    void keyboard(bool *);

public:
    naoCamera(const string&);
    void cameraCalibration();
    void showImages();
    cv::Mat cameraMatrix, distCoeffs;
    void recordDataSet();
};

#endif // GETIMAGES_HPP
