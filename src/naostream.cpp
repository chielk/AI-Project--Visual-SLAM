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

// OpenCV includes.
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

	cv::Mat K; // TODO: store calibration matrix in K

	// Startup the webcam listener
	AL::ALVideoDeviceProxy *camProxy = new AL::ALVideoDeviceProxy( robotIP );
	std::string clientName = camProxy->subscribe( name, AL::kQVGA, AL::kYuvColorSpace, 30 );	
    camProxy->setActiveCamera(clientName, AL::kTopCamera );

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
		std::vector<cv::DMatch> matches;
		matcher.match( current_descriptors, previous_descriptors, matches );

		double max_dist = 0; 
		double min_dist = 100;

		// Quick calculation of max and min distances between keypoints
		for ( int i = 0; i < current_descriptors.rows; i++ ) { 
			double dist = matches[i].distance;
			if ( dist < min_dist ) min_dist = dist;
			if ( dist > max_dist ) max_dist = dist;
		}

		// Find the good matches and calculate centroids
		std::vector<cv::DMatch> good_matches;

		cv::Point2f current_centroid( 0, 0 );
		cv::Point2f previous_centroid( 0, 0 );

		for ( int i = 0; i < current_descriptors.rows; i++ ) { 
			if ( matches[i].distance < 2 * min_dist ) {
				current_centroid  += current_keypoints[matches[i].queryIdx].pt;
				previous_centroid += previous_keypoints[matches[i].trainIdx].pt;

				good_matches.push_back( matches[i] );
			}
		}

		// Normalize the centroids
		current_centroid.x /= matches.size();
		current_centroid.y /= matches.size();

		previous_centroid.x /= matches.size();
		previous_centroid.y /= matches.size();

		double current_scaling = 0;
		double previous_scaling = 0;
		std::vector<cv::DMatch>::iterator i;

		for ( i = good_matches.begin(); i != good_matches.end(); i++ ) {
			cv::Point2f * cp = &current_keypoints[i->queryIdx].pt;
			cv::Point2f * pp = &previous_keypoints[i->trainIdx].pt;

			*cp -= current_centroid;
			*pp -= previous_centroid;

			current_scaling += sqrt( cp->dot( *cp ) );
			previous_scaling += sqrt( pp->dot( *pp ) );
		}

		// Enforce mean distance sqrt( 2 ) from origin
		current_scaling  = sqrt( 2.0 ) * (double) good_matches.size() / current_scaling;
		previous_scaling = sqrt( 2.0 ) * (double) good_matches.size() / previous_scaling;

		// Scale features and store the points
		std::vector<cv::Point2f> current_points, previous_points;

		for ( i = good_matches.begin(); i != good_matches.end(); i++ ) {
			current_points.push_back( current_keypoints[i->queryIdx].pt * current_scaling );
			previous_points.push_back( previous_keypoints[i->trainIdx].pt * previous_scaling );
		}

		// Compute transformation matrices
		cv::Mat current_T, previous_T;
		current_T = ( cv::Mat_<double>(3,3) << 
			current_scaling, 0,               -current_scaling * current_centroid.x, 
			0,               current_scaling, -current_scaling * current_centroid.y, 
			0,               0,               1 
		);

		previous_T = ( cv::Mat_<double>(3,3) << 
			previous_scaling, 0,                -previous_scaling * previous_centroid.x, 
			0,                previous_scaling, -previous_scaling * previous_centroid.y, 
			0,                0,                1 
		);

		// Compute fundamental matrix
		cv::Mat F = cv::findFundamentalMat( previous_points, current_points );
		F = current_T.t() * F * previous_T;

		// Compute essential matrix
		cv::Mat E, U, S, V, T, Ra, Rb, W, Z;
		E = K.inv().t() * F * K;

		cv::Mat U, W, V;
		cv::SVD::compute( E, U, W, V );
		W.at<double>( 2, 0 ) = 0;
		E = U * W.diag() * V.t(); // Possible transpose error

		// Hartley matrices
		W = ( cv::Mat_<double>(3,3) << 0,-1, 0, 1, 0, 0, 0, 0, 1 );
		Z = ( cv::Mat_<double>(3,3) << 0, 1, 0,-1, 0, 0, 0, 0, 0 );

		// Compute R and T
		cv::SVD::compute( E, U, S, V );
		T = U * Z * U.t();
		Ra = U * W * V.t(); // Possible transpose error
		Rb = U * W.t() * V.t(); // Possible transpose error

		cv::Vec3f t = cv::Vec3f( T.at<int>( cv::Point2d(2,1) ), T.at<int>(0,2), T.at<int>(1,0) );

		// Assure determinant is positive
		if ( cv::determinant( Ra ) < 0 ) Ra = -Ra;
		if ( cv::determinant( Rb ) < 0 ) Rb = -Rb;

		// At this point there are 4 possible solutions.
		// Use majority vote to decide winner



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
