/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
 */

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

typedef std::vector<cv::KeyPoint> KeyPointVector;

#define RED cv::Scalar( 0, 0, 255 )

int main( int argc, char* argv[] ) {
    if (argc < 2) {
        std::cerr << "Usage 'getimages robotIp'" << std::endl;
        return 1;
    }

    const std::string robotIP( argv[1] );
	std::string name = "brisk_test";

	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "BRISK" );

	AL::ALVideoDeviceProxy *camProxy = new AL::ALVideoDeviceProxy( robotIP );
	std::string clientName = camProxy->subscribe( name, AL::kVGA, AL::kYuvColorSpace, 30 );
    camProxy->setActiveCamera(clientName, AL::kTopCamera );

    while ( (char) cv::waitKey( 30 ) != 27 ) {
		cv::Mat frame = cv::Mat( cv::Size( 640, 480 ), CV_8UC1 );

        AL::ALValue img = camProxy->getImageRemote( clientName );
		frame.data = (uchar*) img[6].GetBinary();
        camProxy->releaseImage( clientName );
        
		// Detect features
		KeyPointVector kpv;
		detector->detect( frame, kpv );

		KeyPointVector::iterator i;
		for ( i = kpv.begin(); i != kpv.end(); i++ ) {
			cv::circle( frame, i->pt, (int) i->response, RED );
		}
		
		std::stringstream ss;
		ss << "Number of features: " << kpv.size();
		cv::putText( frame, ss.str(), cv::Point(20,20), 0, 0.5, RED );

		cv::imshow( "images", frame );
    }

	camProxy->unsubscribe( name );

    return 0;
}

