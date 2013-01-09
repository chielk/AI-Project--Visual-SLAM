/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
 */

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

#include <iostream>
#include <string>

using namespace AL;
using namespace std;



static double computeReprojectionErrors( const vector<vector<cv::Point3f> >& objectPoints,
                                         const vector<vector<cv::Point2f> >& imagePoints,
                                         const vector<cv::Mat>& rvecs,
                                         const vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix ,
                                         const cv::Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

cv::Mat cameraCalibration(const std::string& robotIp)
{
    ALVideoDeviceProxy camProxy(robotIp, 9559);

    //ALMotionProxy motion(robotIp, 9559);

    //ALValue jointName = "Body";
    //ALValue stiffness = 1.0f;
    //ALValue time = 1.0f;
    //motion.stiffnessInterpolation(jointName, stiffness, time);

    //ALValue names = ALValue::array("HeadPitch", "HeadYaw");
    //ALValue angles = ALValue::array(0.0f, 0.0f);
    //motion.setAngles(names, angles, 0.3f);
    //motion.stiffnessInterpolation(jointName, 0.0f, time);

    cv::Size imageSize = cv::Size(640, 480);
    try {
          camProxy.unsubscribe("test");
    }
    catch (const AL::ALError& e) { }
    const std::string clientName = camProxy.subscribe("test", kVGA, kYuvColorSpace, 30);

    cv::Mat imgHeader = cv::Mat(imageSize, CV_8UC1);
    cv::namedWindow("images");

    // calibration params and variables
    bool calibrated = false;
    bool found;

    vector<vector<cv::Point2f> > imagePoints;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size boardSize = cv::Size(5,8);

    float squareSize = 0.027;

    // camera matrix for intrinsic and extrinsic parameters
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 1.0;

    /** Main loop. Exit when pressing ESC.*/
    while ((char) cv::waitKey(30) != 27)
    {
        ALValue img = camProxy.getImageRemote(clientName);
        imgHeader.data = (uchar*) img[6].GetBinary();
        camProxy.releaseImage(clientName);

        vector<cv::Mat> rvecs, tvecs;
        vector<cv::Point2f> pointBuf;

        distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

        found = cv::findChessboardCorners( imgHeader, 
                                           boardSize, 
                                           pointBuf,
                                           CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            // improve the found corners' coordinate accuracy for chessboard
            cv::cornerSubPix( imgHeader,
                              pointBuf,
                              cv::Size(11,11),
                              cv::Size(-1,-1),
                              cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            // size of chessboard squares, compute real points in the object
            imagePoints.push_back(pointBuf);
            vector<vector<cv::Point3f> > objectPoints(1);
            for( int i = 0; i < boardSize.height; ++i )
                for( int j = 0; j < boardSize.width; ++j )
                    objectPoints[0].push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
            objectPoints.resize(imagePoints.size(), objectPoints[0]);

            // Draw the corners.
            cv::drawChessboardCorners( imgHeader, boardSize, cv::Mat(pointBuf), found );

            // Perform calibration based on old values
            double rms;
            if(calibrated)
            {
                rms = cv::calibrateCamera(objectPoints,
                                          imagePoints,
                                          imageSize,
                                          cameraMatrix,
                                          distCoeffs,
                                          rvecs,
                                          tvecs,
                                          CV_CALIB_USE_INTRINSIC_GUESS|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
            } else {
                rms = cv::calibrateCamera(objectPoints,
                                          imagePoints,
                                          imageSize,
                                          cameraMatrix,
                                          distCoeffs,
                                          rvecs,
                                          tvecs,
                                          CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
            }
            cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

            vector<float> reprojErrs;
            double totalAvgErr;

            totalAvgErr = computeReprojectionErrors(objectPoints,
                                                    imagePoints,
                                                    rvecs,
                                                    tvecs,
                                                    cameraMatrix,
                                                    distCoeffs,
                                                    reprojErrs);

            calibrated = (cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs));
        }
        cv::imshow("images", imgHeader);
    }

    /**
    while ((char) cv::waitKey(20) != 27)
    {
        ALValue img = camProxy.getImageRemote(clientName);
        imgHeader.data = (uchar*) img[6].GetBinary();
        camProxy.releaseImage(clientName);

        cv::Mat temp = imgHeader.clone();
        cv::undistort(temp, imgHeader, cameraMatrix, distCoeffs);
        cv::imshow("images", imgHeader);
    }
    **/

    return cameraMatrix;
}

void showImages(const std::string& robotIp)
{
    ALVideoDeviceProxy camProxy(robotIp, 9559);
    ALMotionProxy motion(robotIp, 9559);

    ALValue jointName = "Body";
    ALValue stiffness = 1.0f;
    ALValue time = 1.0f;
    motion.stiffnessInterpolation(jointName, stiffness, time);

    ALValue names = ALValue::array("HeadPitch", "HeadYaw");
    ALValue angles = ALValue::array(-1.5f, 1.5f);
    motion.setAngles(names, angles, 0.3f);

    jointName = "HeadYaw";
    ALValue targetAngles = AL::ALValue::array(-1.5f, 1.5f);
    ALValue targetTimes = AL::ALValue::array(3.0f, 6.0f);
    bool isAbsolute = true;

    try {
          camProxy.unsubscribe("test");
    }
    catch (const AL::ALError& e) { }

    motion.post.walkTo(0.2, 0, 0);

    const std::string clientName = camProxy.subscribe("test", kVGA, kYuvColorSpace, 30);
    cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC1);
    cv::namedWindow("images");
  
    while (motion.walkIsActive())
    {
        if(motion.getAngles(jointName, true).at(0) > 1.4f)
        {
            motion.post.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);
        }
        int counter = 1;

        /** Main loop. Exit when pressing ESC.*/
        while ((char) cv::waitKey(30) != 27)
        {
            ALValue img = camProxy.getImageRemote(clientName);
            imgHeader.data = (uchar*) img[6].GetBinary();
            camProxy.releaseImage(clientName);

            char filename[50];
            sprintf(filename, "./images/images%d.png", counter);

            cv::imshow("images", imgHeader);
            cv::imwrite(filename, imgHeader);
            counter++;
        }
    camProxy.unsubscribe(clientName);
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage 'getimages robotIp'" << std::endl;
        return 1;
    }

    const std::string robotIp(argv[1]);

    cv::Mat meanIntrinsic = cv::Mat(3, 3, CV_64FC1);
    cv::Mat cameraMatrix;
    int N = 5;

    try
    {
        for (int n = 0; n < N; n++)
        {
            cameraMatrix = cameraCalibration(robotIp);
            for (int i = 0 ; i < 3; i ++)
            {
                for (int j = 0 ; j < 3; j ++)
                {
                    meanIntrinsic.at<double>(i,j) += cameraMatrix.at<double>(i,j);
                }
            }
        }


        for (int i = 0 ; i < 3; i ++)
        {
            for (int j = 0 ; j < 3; j ++)
            {
                cout << meanIntrinsic.at<double>(i,j) / (float)N << "\t";
            }
            cout << endl;
        }

    }
    catch (const AL::ALError& e)
    {
        std::cerr << "Caught exception " << e.what() << std::endl;
    }

    return 0;
}

