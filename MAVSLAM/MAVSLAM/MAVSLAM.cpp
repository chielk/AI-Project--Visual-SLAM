// MAVSLAM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>

typedef std::vector<cv::KeyPoint> KeyPointVector;
typedef std::vector<cv::Mat> ImageVector;

#define RED cv::Scalar( 0, 0, 255 )

int _tmain( int argc, _TCHAR* argv[] ) {
	cv::VideoCapture cap( 0 ); // open the default camera
    if ( !cap.isOpened() )  // check if we succeeded
        return -1;

    cv::namedWindow( "window", 1 );
	cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "BRISK" );
	/*
	ImageVector images;
	int count = 0;
	*/

	for (;;)
    {
        cv::Mat frame;
		cap >> frame;

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
		cv::imshow( "window", frame );

        if ( cv::waitKey( 100 ) != -1 ) break;

		/*
		//cv::cvtColor( frame, edges, CV_BGR2BGR );
        //cv::GaussianBlur( edges, edges, cv::Size( 7, 7 ), 1.5, 1.5 );
        //cv::Canny( edges, edges, 0, 30, 3 );

		
		
		// Wait for us to press key
		while( cv::waitKey( 100 ) == -1 )
		{
			cap >> frame;
			cv::imshow( "window", frame );
		}

		// Get a new frame from the camera for stitching
		// TODO: replace with getter from nao or files
        cap >> frame; // get a new frame from camera

		images.push_back( frame.clone() );
		if ( ++count == 3 ) break;*/
    }
	/*
	cap.release();

	cv::Mat stitch;
	cv::Stitcher stitcher = cv::Stitcher::createDefault( false );

	cv::Stitcher::Status status = stitcher.stitch( images, stitch );
	
	if (status != cv::Stitcher::OK)
    {
        std::cout << "Can't stitch images, error code = " << int(status) << std::endl;
        return -1;
    }
	//stitcher.composePanorama( images, stitch );

	cv::imwrite( "henk.jpg", stitch );
	*/

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}