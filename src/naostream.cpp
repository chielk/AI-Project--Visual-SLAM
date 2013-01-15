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

#define RED cv::Scalar( 0, 0, 255 )

typedef std::vector<cv::KeyPoint> KeyPointVector;

int main( int argc, char* argv[] ) {
    if ( argc < 2 ) {
        std::cerr << "Usage 'getimages robotIp'" << std::endl;
        return 1;
    }

    const std::string robotIP( argv[1] );

	std::string name = "brisk_test";

    cv::SiftDescriptorExtractor extractor;
	cv::Mat current_descriptors, previous_descriptors;
	KeyPointVector current_keypoints, previous_keypoints;
	cv::Mat current_frame = cv::Mat( cv::Size( 320, 240 ), CV_8UC1 );
	cv::Mat previous_frame = cv::Mat( cv::Size( 320, 240 ), CV_8UC1 );

    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "SIFT" );

	// Startup the webcam listener
	AL::ALVideoDeviceProxy *camProxy = new AL::ALVideoDeviceProxy( robotIP );
	std::string clientName = camProxy->subscribe( name, AL::kQVGA, AL::kYuvColorSpace, 30 );	
    camProxy->setActiveCamera(clientName, AL::kBottomCamera );

	// First frame processing
    AL::ALValue img = camProxy->getImageRemote( clientName );
	previous_frame.data = (uchar*) img[6].GetBinary();
    camProxy->releaseImage( clientName );
        
	// Detect features
	detector->detect( previous_frame, previous_keypoints );
	extractor.compute( previous_frame, previous_keypoints, previous_descriptors );

    while ( (char) cv::waitKey( 30 ) == -1 ) {
		// Retrieve an image
		AL::ALValue img = camProxy->getImageRemote( clientName );
		current_frame.data = (uchar*) img[6].GetBinary();
        camProxy->releaseImage( clientName );
        
		// Detect features
		detector->detect( current_frame, current_keypoints );
		extractor.compute( current_frame, current_keypoints, current_descriptors );

		// Match descriptor vectors using FLANN matcher
		cv::FlannBasedMatcher matcher;
		std::vector< cv::DMatch > matches;
		matcher.match( current_descriptors, previous_descriptors, matches );

		double max_dist = 0; 
		double min_dist = 100;

		// Quick calculation of max and min distances between keypoints
		for( int i = 0; i < current_descriptors.rows; i++ ) { 
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}

		// Find the good matches (i.e. whose distance is less than 2*min_dist )
		std::vector<cv::DMatch> good_matches;

		for ( int i = 0; i < current_descriptors.rows; i++ ) { 
			if ( matches[i].distance < 2 * min_dist ) { 
				good_matches.push_back( matches[i] ); 
			}
		}

		// Draw only "good" matches
		cv::Mat img_matches;
		drawMatches( 
			current_frame, current_keypoints, previous_frame, previous_keypoints,
			good_matches, img_matches, cv::Scalar::all( -1 ), cv::Scalar::all( -1 ),
			std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS 
		);

		// Show detected matches
		imshow( "Good Matches", img_matches );

		// Assign current values to the previous ones, for the next iteration
		previous_keypoints = current_keypoints;
		previous_frame = current_frame;
		previous_descriptors = current_descriptors;
    }

	camProxy->unsubscribe( name );

    return 0;
}
