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

#include <iostream>
#include <string>

using namespace AL;

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

    /** Subscribe a client image requiring 640x480 and RGB colorspace.*/
    const std::string clientName = camProxy.subscribe("test", kVGA, kYuvColorSpace, 30);

    /** Create an cv::Mat header to wrap into an opencv image.*/
    cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC1);
  
    /** Create a OpenCV window to display the images. */
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

  try
  {
    showImages(robotIp);
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  return 0;
}

