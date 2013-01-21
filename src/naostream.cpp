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
#include "inputsource.hpp"

#define RED cv::Scalar( 0, 0, 255 )

typedef std::vector<cv::KeyPoint> KeyPointVector;

int main( int argc, char* argv[] ) {
    if ( argc < 3 ) {
        std::cerr << "Usage" << argv[0] << " '(-n robotIp|-f folderName)'" << std::endl;
        return 1;
    }

    InputSource *inputSource;

    if (std::string(argv[1]) == "-n")
    {
        const std::string robotIp( argv[2] );
        inputSource = new NaoInput( robotIp );
    }
    else if(std::string(argv[1]) == "-f" )
    {
        const std::string folderName(argv[2]);
        inputSource = new FileInput( folderName );
    } else {
        std::cout << "Wrong use of command line arguments." << std::endl;
        return 1;
    }

    std::string name = "brisk_test";
    cv::Size imageSize = cv::Size(640, 480);
    int channels = CV_8UC3;

    cv::Mat current_descriptors, previous_descriptors;
    KeyPointVector current_keypoints, previous_keypoints;

    cv::BRISK brisk(60, 4, 1.0f);
    brisk.create("BRISK");

    //cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "BRISK" );
    //cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create( "BRISK" );

    //cv::Ptr<cv::FeatureDetector> detector = cv::Algorithm::create<cv::FeatureDetector>( "Feature2D.BRISK" );
    //cv::Ptr<cv::DescriptorExtractor> extractor = cv::Algorithm::create<cv::DescriptorExtractor>( "Feature2D.BRISK" );

    // Load calibrationmatrix K (and distortioncoefficients while we're at it).
    cv::Mat K;
    cv::Mat distortionCoeffs;
    if ( !loadSettings( K, distortionCoeffs ) )
        return 1;

    // Get the previous frame
    Frame current_frame;
    Frame previous_frame;

    inputSource->getFrame(previous_frame);

    // Detect features
    brisk.detect( previous_frame.img, previous_keypoints );    
    brisk.compute( previous_frame.img, previous_keypoints, previous_descriptors );

    // Hartley matrices
    cv::Mat hartley_W = ( cv::Mat_<double>(3,3) << 0,-1, 0, 1, 0, 0, 0, 0, 1 );
    cv::Mat hartley_Z = ( cv::Mat_<double>(3,3) << 0, 1, 0,-1, 0, 0, 0, 0, 0 );

    while ( (char) cv::waitKey( 30 ) == -1 ) {
        // Retrieve an image
        if(!inputSource->getFrame(current_frame))
        {
            std::cout << "Can not read the next frame." << std::endl;
            break;
        }
        if (!current_frame.img.data)
        {
            std::cerr << "No image found." << std::endl;
            return 1;
        }

        // Detect features
        brisk.detect( current_frame.img, current_keypoints );

        // TODO : What if zero features found?
        if(previous_keypoints.size() == 0)
        {
            continue;
        }

        brisk.compute( current_frame.img, current_keypoints, current_descriptors );

        // Match descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20,10,2));
        std::vector<cv::DMatch> matches;

        matcher.match( current_descriptors, previous_descriptors, matches );

        double max_dist = 0;
        double min_dist = 100;
        double dist;
        std::vector<cv::DMatch>::iterator match_it;

        // Quick calculation of max and min distances between keypoints
        for ( match_it = matches.begin(); match_it != matches.begin() + current_descriptors.rows; match_it++ ) {
            dist = match_it->distance;
            if ( dist < min_dist ) min_dist = dist;
            else if ( dist > max_dist ) max_dist = dist;
        }

        // Find the good matches and calculate centroids
        std::vector<cv::DMatch> good_matches;
        cv::Point2f current_centroid( 0, 0 );
        cv::Point2f previous_centroid( 0, 0 );

        double double_min_dist = 2*(min_dist + 20);
        for ( match_it = matches.begin(); match_it != matches.begin() + current_descriptors.rows; match_it++ ) {
            if ( match_it->distance <= double_min_dist ) {
                current_centroid  += current_keypoints[match_it->queryIdx].pt;
                previous_centroid += previous_keypoints[match_it->trainIdx].pt;

                good_matches.push_back( *match_it );
            }
        }

        // Draw only "good" matches
        cv::Mat img_matches;
        cv::drawMatches(
            current_frame.img, current_keypoints, previous_frame.img, previous_keypoints,
            good_matches, img_matches, cv::Scalar::all( -1 ), cv::Scalar::all( -1 ),
            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
            );

        // Show detected matches
        imshow( "Good Matches", img_matches );
        imwrite("some.png", img_matches);


        // Normalize the centroids
        int matchesSize = matches.size();
        current_centroid.x /= matchesSize;
        current_centroid.y /= matchesSize;
        previous_centroid.x /= matchesSize;
        previous_centroid.y /= matchesSize;

        double current_scaling = 0;
        double previous_scaling = 0;

        cv::Point2f *cp;
        cv::Point2f *pp;
        for ( match_it = good_matches.begin(); match_it < good_matches.end(); match_it++ ) {
            cp = &current_keypoints[match_it->queryIdx].pt;
            pp = &previous_keypoints[match_it->trainIdx].pt;

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

        for ( match_it = good_matches.begin(); match_it != good_matches.end(); match_it++ ) {
            current_points.push_back( current_keypoints[match_it->queryIdx].pt * current_scaling );
            previous_points.push_back( previous_keypoints[match_it->trainIdx].pt * previous_scaling );
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
        cv::Mat E, S, T, Ra, Rb;
        E = K.inv().t() * F * K;

        // Enforce rank 2-ness
        cv::Mat U (3,3, CV_32F);
        cv::Mat W, Vt, t;
        cv::SVD::compute( E, W, U, Vt );

        cv::Mat w = ( cv::Mat_<double>(3,3) << W.at<double>(0,0), 0.0, 0.0,
            0.0, W.at<double>(1,0), 0.0,
            0.0, 0.0, 0.0 );
        E = U * w * Vt;

        // Compute R and T
        cv::SVD::compute( E, S, U, Vt );
        Ra = U * hartley_W * Vt;         // Possible transposed error
        Rb = U * hartley_W.t() * Vt;
        t = ( cv::Mat_<double>(3,1) << U.at<double>(0,2),
                                       U.at<double>(1,2),
                                       U.at<double>(2,2) );

        std::cout << matrixToString(Ra) << "\n\n" << matrixToString(Rb) << std::endl;

        // Assure determinant is positive
        if ( cv::determinant( Ra ) < 0 ) Ra = -Ra;
        if ( cv::determinant( Rb ) < 0 ) Rb = -Rb;

        // At this point there are 4 possible solutions.
        // Use majority vote to decide winner

        // create vector containing all 4 solutions
        cv::Mat possible_projection[4];
        cv::hconcat( Ra,  t , possible_projection[0] ); 
        cv::hconcat( Ra, -1 * t , possible_projection[1] ); 
        cv::hconcat( Rb,  t , possible_projection[2] ); 
        cv::hconcat( Rb, -1 * t , possible_projection[3] );

        int max_inliers = 0;
        int num_inliers = 0;
        cv::Mat best_transform;

        cv::Mat X = cv::Mat( 4, good_matches.size(), CV_64F, cv::Scalar( 1 ) );

        for ( int i = 0; i < 4; i++ ) {
            // Get the number of inliers
            cv::Mat J = ( cv::Mat_<double>( 4, 4 ) << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 );
            cv::Mat U, S, V;

            cv::Mat P1;
            cv::hconcat( K, (cv::Mat) ( cv::Mat_<double>(3,1) << 0, 0, 0 ), P1 );

            // Begin inner loops
            for ( int m = 0; m < (int) good_matches.size(); m++ ) {
                cv::Point pp = previous_keypoints[good_matches[m].trainIdx].pt;
                cv::Point cp = current_keypoints[good_matches[m].queryIdx].pt;

                for ( int j = 0; j < 4; j++ ) {
                    J.at<double>(0, j) = P1.at<double>(2, j) * pp.x - P1.at<double>(0, j);
                    J.at<double>(1, j) = P1.at<double>(2, j) * pp.y - P1.at<double>(1, j);
                    J.at<double>(2, j) = possible_projection[i].at<double>(2, j) * cp.x - possible_projection[i].at<double>(0, j);
                    J.at<double>(3, j) = possible_projection[i].at<double>(2, j) * cp.y - possible_projection[i].at<double>(1, j);
                }
                cv::SVD::compute(J, S, U, V);

                X.at<double>( 0, m ) = V.at<double>( 0, 2 );
                X.at<double>( 1, m ) = V.at<double>( 1, 2 );
                X.at<double>( 2, m ) = V.at<double>( 2, 2 );

                //cv::hconcat( V(cv::Range(0,3), cv::Range(3,3)), X );
            }

            cv::Mat AX1;
            cv::Mat BX1;

            AX1 = P1 * X;
            BX1 = possible_projection[i] * X;
            num_inliers = 0;

            // Calculating inliers
            for (int j = 0; j < good_matches.size(); j++ ) {
                if ( AX1.at<double>(2, j) * X.at<double>(3, j)  > 0 && BX1.at<double>(2, j) * X.at<double>(3, j) > 0 ) {
                    num_inliers++;
                }
            }

            if ( num_inliers > max_inliers ) {
                max_inliers = num_inliers;
                best_transform = possible_projection[i];
            }
        }

        std::cout << matrixToString( best_transform ) << std::endl;
        std::cout << matrixToString(X.t()) << std::endl;

        // SOLVE THEM SCALE ISSUES for m = 1;
        cv::Mat A (2 * good_matches.size(), 1, CV_32F, 0.0f);
        cv::Mat b (2 * good_matches.size(), 1, CV_32F, 0.0f);
        cv::Point p;

        cv::Mat r1(best_transform(cv::Range(0,1), cv::Range(0,3)));
        cv::Mat r2(best_transform(cv::Range(1,2), cv::Range(0,3)));
        cv::Mat r3(best_transform(cv::Range(2,3), cv::Range(0,3)));
        cv::Mat temp1;
        cv::Mat temp2;
        cv::Mat point3D;

        double s_t_u = best_transform.at<double>(0,3);
        double s_t_v = best_transform.at<double>(1,3);
        double s_t_w = best_transform.at<double>(1,3);

        int i = 0;
        for ( match_it = good_matches.begin(); match_it != good_matches.end(); match_it++ ) {
            p = current_keypoints[match_it->queryIdx].pt;
            point3D = X(cv::Range(0,3), cv::Range(i/2, i/2+1));

            cv::subtract(r1, r3 * p.x, temp1);
            cv::subtract(r2, r3 * p.y, temp2);

            // Method 1
            A.at<float>(i,1) = s_t_w * p.x - s_t_u;
            b.at<float>(i,1) = ((cv::Mat)(temp1 * point3D)).at<float>(0,0);

            // Method 2
            A.at<float>(i+1,1) = s_t_w * p.y - s_t_v;
            b.at<float>(i+1,1) = ((cv::Mat)(temp2 * point3D)).at<float>(0,0);

            // Together, these comprise method 3
            i = i + 2;
        }

        std::cout<< A.size().height << " " << A.size().width << std::endl;
        std::cout<< b.size().height << " " << b.size().width << std::endl;

        A = (A.t() * A).inv() * A.t();

        std::cout << matrixToString((cv::Mat)(A*b)) << std::endl;

        double s = ((cv::Mat)(A * b)).at<double>(0,0);
        std::cout << "Found scale difference: " << s << std::endl;

        // Assign current values to the previous ones, for the next iteration
        previous_keypoints = current_keypoints;
        previous_frame = current_frame;
        previous_descriptors = current_descriptors;
    }

    return 0;
}
