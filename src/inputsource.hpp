#ifndef INPUTSOURCE_H
#define INPUTSOURCE_H

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <stdio.h>

typedef struct
{
    std::vector<float> camPosition;
    cv::Mat img;
} Frame;

class InputSource
{
public:
    virtual bool getFrame(Frame &frame) = 0;
};

class NaoInput : public InputSource
{
    std::string clientName;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    std::vector<float> initialCameraPosition;

    AL::ALVideoDeviceProxy *camProxy;

    void subscribe(std::string nmotProxyame, int cameraId);
    void unsubscribe(std::string &name);
    void init(const std::string &robotIp,
              std::string name,
              int cameraId,
              cv::Mat &cameraMatrix,
              cv::Mat &distortionCoeffs);

public:
    NaoInput(const std::string &robotIp);
    NaoInput(const std::string &robotIp,
             std::string name,
             int cameraId,
             cv::Mat &cameraMatrix,
             cv::Mat &distortionCoeffs);
    ~NaoInput();
    bool getFrame(Frame &frame);

    // public property so naocontroller can refer to it
    AL::ALMotionProxy *motProxy;
};

class FileInput : public InputSource
{
    int index;
    std::string foldername;
    std::ifstream odometryFile;
public:
    FileInput(const std::string foldername);
    ~FileInput();
    bool getFrame(Frame &frame);
};

void undistortImage(cv::Mat &image, cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs);

typedef struct
{
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Size cvImsize;
    int imsize;
    int colorspace;
} config;

void saveSettings(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs);
bool loadSettings(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs);

std::string matrixToString(cv::Mat);


#endif // INPUTSOURCE_H
