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
#define EPSILON 0.0001

typedef std::vector<cv::KeyPoint> KeyPointVector;




bool DecomposeEtoRandT( cv::Matx33d &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t ) {
    //Using HZ E decomposition
    cv::SVD svd(E, cv::SVD::MODIFY_A);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabsf( svd.w.at<double>( 0 ) / svd.w.at<double>( 1 ) );
    std::cout << svd.w << std::endl;
    if ( singular_values_ratio > 1.0 ) {
        singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    }
    if ( singular_values_ratio < 0.4 ) {
        std::cout << "singular values are too far apart\n" << std::endl;
        return false;
    }

    cv::Matx33d W(  0, -1,  0,	//HZ 9.13
                    1,  0,  0,
                    0,  0,  1 );
    cv::Matx33d Wt( 0,  1,  0,
                   -1,  0,  0,
                    0,  0,  1 );

    R1 = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
    R2 = svd.u * cv::Mat(Wt) * svd.vt; //HZ 9.19
    t = svd.u.col(2); //u3

    return true;
}


//From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
//
// Arguments:
//     u1 and u2: homogenous image point (u,v,1)
//     P1 and P2: Camera matrices
cv::Mat_<double> LinearLSTriangulation( cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2 ) {
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    // cout << "u " << u <<", u1 " << u1 << endl;
    // Matx<double,6,4> A; //this is for the AX=0 case, and with linear dependence..
    // A(0) = u.x*P(2)-P(0);
    // A(1) = u.y*P(2)-P(1);
    // A(2) = u.x*P(1)-u.y*P(0);
    // A(3) = u1.x*P1(2)-P1(0);
    // A(4) = u1.y*P1(2)-P1(1);
    // A(5) = u1.x*P(1)-u1.y*P1(0);
    // Matx43d A; //not working for some reason...
    // A(0) = u.x*P(2)-P(0);
    // A(1) = u.y*P(2)-P(1);
    // A(2) = u1.x*P1(2)-P1(0);
    // A(3) = u1.y*P1(2)-P1(1);
    cv::Matx43d A(  u1.x * P1(2,0) - P1(0,0), u1.x * P1(2,1) - P1(0,1), u1.x * P1(2,2) - P1(0,2),
                    u1.y * P1(2,0) - P1(1,0), u1.y * P1(2,1) - P1(1,1), u1.y * P1(2,2) - P1(1,2),
                    u2.x * P2(2,0) - P2(0,0), u2.x * P2(2,1) - P2(0,1), u2.x * P2(2,2) - P2(0,2),
                    u2.y * P2(2,0) - P2(1,0), u2.y * P2(2,1) - P2(1,1), u2.y * P2(2,2) - P2(1,2)
                 );
    cv::Matx41d B( -( u1.x * P1(2,3) - P1(0,3) ),
                   -( u1.y * P1(2,3) - P1(1,3) ),
                   -( u2.x * P2(2,3) - P2(0,3) ),
                   -( u2.y * P2(2,3) - P2(1,3) )
                 );

    cv::Mat_<double> X;
    cv::solve(A,B,X,cv::DECOMP_SVD);

    return X;
}


/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Matx31d IterativeLinearLSTriangulation(cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2	) {
    double wi1 = 1;
    double wi2 = 1;

    cv::Matx41d X;

    for ( int i = 0; i < 10; i++ ) {
        // Hartley suggests 10 iterations at most
        cv::Mat_<double> X_ = LinearLSTriangulation( u1, P1, u2, P2 );
        X = cv::Matx41d( X_(0), X_(1), X_(2), 1.0 );

        // Recalculate weights
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
        double p2x2 = cv::Mat_<double>(cv::Mat_<double>(P2).row(2)*X)(0);

        // Breaking point
        if ( fabsf( wi1 - p2x1 ) <= EPSILON && fabsf( wi2 - p2x2 ) <= EPSILON ) break;

        wi1 = p2x1;
        wi2 = p2x2;

        // Reweight equations and solve
        cv::Matx43d A( ( u1.x * P1(2,0) - P1(0,0) ) / wi1, ( u1.x * P1(2,1) - P1(0,1) ) / wi1, ( u1.x * P1(2,2) - P1(0,2) ) / wi1,
                       ( u1.y * P1(2,0) - P1(1,0) ) / wi1, ( u1.y * P1(2,1) - P1(1,1) ) / wi1, ( u1.y * P1(2,2) - P1(1,2) ) / wi1,
                       ( u2.x * P2(2,0) - P2(0,0) ) / wi2, ( u2.x * P2(2,1) - P2(0,1) ) / wi2, ( u2.x * P2(2,2) - P2(0,2) ) / wi2,
                       ( u2.y * P2(2,0) - P2(1,0) ) / wi2, ( u2.y * P2(2,1) - P2(1,1) ) / wi2, ( u2.y * P2(2,2) - P2(1,2) ) / wi2
                 );
        cv::Matx41d B( -( u1.x * P1(2,3) - P1(0,3) ) / wi1,
                       -( u1.y * P1(2,3) - P1(1,3) ) / wi1,
                       -( u2.x * P2(2,3) - P2(0,3) ) / wi2,
                       -( u2.y * P2(2,3) - P2(1,3) ) / wi2
                     );

        cv::solve( A, B, X_, cv::DECOMP_SVD );
        X = cv::Matx41d( X_(0), X_(1), X_(2), 1.0 );
    }

    return cv::Matx31d( X(0), X(1), X(2) );
}



int main( int argc, char* argv[] ) {
    if ( argc < 3 ) {
        std::cerr << "Usage" << argv[0] << " '(-n robotIp|-f folderName)'" << std::endl;
        return 1;
    }

    InputSource *inputSource;

    if ( std::string(argv[1]) == "-n" ) {
        const std::string robotIp( argv[2] );
        inputSource = new NaoInput( robotIp );
    } else if ( std::string(argv[1]) == "-f" ) {
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
    if ( !loadSettings( K, distortionCoeffs ) ) {
        return 1;
    }

    // Get the previous frame
    Frame current_frame;
    Frame previous_frame;

    inputSource->getFrame( previous_frame );

    // Detect features
    brisk.detect( previous_frame.img, previous_keypoints );    
    brisk.compute( previous_frame.img, previous_keypoints, previous_descriptors );

    // Hartley matrices
    cv::Matx33d hartley_W( 0,-1, 0, 
                           1, 0, 0,
                           0, 0, 1 );

    cv::Matx33d hartley_Z( 0, 1, 0, 
                          -1, 0, 0,
                           0, 0, 0 );

    while ( (char) cv::waitKey( 30 ) == -1 ) {
        // Retrieve an image
        if ( !inputSource->getFrame( current_frame ) ) {
            std::cout << "Can not read the next frame." << std::endl;
            break;
        }
        if ( !current_frame.img.data ) {
            std::cerr << "No image found." << std::endl;
            return 1;
        }

        // Detect features
        brisk.detect( current_frame.img, current_keypoints );

        // TODO : What if zero features found?
        if ( previous_keypoints.size() == 0 ) {
            continue;
        }

        brisk.compute( current_frame.img, current_keypoints, current_descriptors );

        // Match descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher( new cv::flann::LshIndexParams( 20, 10, 2 ) );
        std::vector<cv::DMatch> matches;

        matcher.match( current_descriptors, previous_descriptors, matches );

        // Quick calculation of centroid
        std::vector<cv::DMatch>::iterator match_it;
        cv::Point2f current_centroid(0,0);
        cv::Point2f previous_centroid(0,0);

        std::vector<cv::Point2f> current_points, previous_points;
        cv::Point2f cp;
        cv::Point2f pp;

        for ( match_it = matches.begin(); match_it != matches.begin() + current_descriptors.rows; match_it++ ) {
            cp = current_keypoints[match_it->queryIdx].pt;
            pp = previous_keypoints[match_it->trainIdx].pt;

            current_centroid.x += cp.x;
            current_centroid.y += cp.y;
            current_points.push_back( cp );

            previous_centroid.x += pp.x;
            previous_centroid.y += pp.y;
            previous_points.push_back( pp );
        }

        // Normalize the centroids
        int matchesSize = matches.size();
        current_centroid.x /= matchesSize;
        current_centroid.y /= matchesSize;
        previous_centroid.x /= matchesSize;
        previous_centroid.y /= matchesSize;

        double current_scaling = 0;
        double previous_scaling = 0;

        cv::Point2f *cp_ptr, *pp_ptr;
        for ( int i = 0; i < matches.size(); i++ ) {
            cp_ptr = &current_points[i];
            pp_ptr = &previous_points[i];

            *cp_ptr -= current_centroid;
            *pp_ptr -= previous_centroid;

            current_scaling += sqrt( cp_ptr->dot( *cp_ptr ) );
            previous_scaling += sqrt( pp_ptr->dot( *pp_ptr ) );
        }


        // Enforce mean distance sqrt( 2 ) from origin
        current_scaling  = sqrt( 2.0 ) * (double) matches.size() / current_scaling;
        previous_scaling = sqrt( 2.0 ) * (double) matches.size() / previous_scaling;

        // Compute transformation matrices
        cv::Matx33d current_T( current_scaling, 0,               -current_scaling * current_centroid.x,
                               0,               current_scaling, -current_scaling * current_centroid.y,
                               0,               0,               1 
                             );

        cv::Matx33d previous_T ( previous_scaling, 0,                -previous_scaling * previous_centroid.x,
                                 0,                previous_scaling, -previous_scaling * previous_centroid.y,
                                 0,                0,                1
                               );

        for ( int i = 0; i < matches.size(); i++ ) {
            current_points[i]  *= current_scaling;
            previous_points[i] *= previous_scaling;
        }

        std::vector<cv::Point2f> current_points_good, previous_points_good;
        std::vector<cv::DMatch> good_matches;

        // Find the fundamental matrix.
        double minVal, maxVal;
        cv::minMaxIdx( previous_points, &minVal, &maxVal );

        std::vector<uchar> status( matches.size() );
        cv::Mat F = cv::findFundamentalMat( previous_points, current_points, status, cv::FM_RANSAC, 0.006 * maxVal, 0.99 );

        // Scale up again
        F = current_T.t() * F * previous_T;

        // Reject outliers
        for ( int i = 0; i < matchesSize; i++ ) {
            if( status[i] ) {
                current_points_good.push_back( current_keypoints[matches[i].queryIdx].pt );
                previous_points_good.push_back( previous_keypoints[matches[i].trainIdx].pt );

                // Needed for displaying inliers
                good_matches.push_back( matches[i] );
            }
        }
        std::cout << "Matches before pruning: " << matchesSize << "\n" << "Matches after: " << good_matches.size() << std::endl;


        //// Check : These values should be 0 (or close to it)
        //for(int i = 0; i < current_points_good.size(); i++) {
        //    std::cout << (cv::Mat) cv::Matx31d(current_points_good[i].x, current_points_good[i].y, 1).t() * F * (cv::Mat) cv::Matx31d(previous_points_good[i].x, previous_points_good[i].y, 1) << std::endl;
        //}

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

        // Compute essential matrix
        std::cout << "Fundamental:\n" << F <<  std::endl;

        cv::Matx33d E (cv::Mat(K.t() * F * K));
        std::cout << "\nEssential:\n" << E <<  std::endl;

        // Estimation of projection matrix
        cv::Mat R1, R2, t;
        if( !DecomposeEtoRandT( E, R1, R2, t ) ) {
            return -1;
        }

        // Check correctness
        if( cv::determinant(R1) < 0 ) R1 = -R1;
        if( cv::determinant(R2) < 0 ) R2 = -R2;

        cv::Mat possible_projections[4];
        cv::hconcat( R1,  t, possible_projections[0] );
        cv::hconcat( R1, -t, possible_projections[1] );
        cv::hconcat( R2,  t, possible_projections[2] );
        cv::hconcat( R2, -t, possible_projections[3] );

        // Construct matrix [I|0]
        cv::Matx34d P1( 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0 );
        cv::Matx34d P2;

        int max_inliers = 0;
        cv::Mat best_X ( 4, good_matches.size(), CV_64F, cv::Scalar( 1 ) );
        cv::Matx34d best_transform;

        // Loop over possible candidates
        for ( int i = 0 ; i < 4; i++ ) {
            P2 = possible_projections[i];

            cv::Mat X ( 4, good_matches.size(), CV_64F, cv::Scalar( 1 ) );
            int num_inliers = 0;

            // TODO replace by iterator?
            for ( int m = 0; m < (int) good_matches.size(); m++ ) {
                cv::Point3f previous_point_homogeneous( previous_points_good[m].x,
                                                        previous_points_good[m].y,
                                                        1 );
                cv::Point3f current_point_homogeneous( current_points_good[m].x,
                                                       current_points_good[m].y,
                                                       1 );

                cv::Matx31d X_a = IterativeLinearLSTriangulation( 
                    previous_point_homogeneous,	P1,
                    current_point_homogeneous, P2 );

                X.at<double>(0,m) = X_a(0);
                X.at<double>(1,m) = X_a(1);
                X.at<double>(2,m) = X_a(2);

                if ( X.at<double>(0,m) > 0 ) {
                    num_inliers++;
                }
            }
            if ( num_inliers > max_inliers ) {
                best_X = X.clone();
                max_inliers = num_inliers;
                best_transform = P2;
            }
        }

        std::cout << matrixToString(best_X.t()) << std::endl;
        std::cout << best_transform << std::endl;

        // SOLVE THEM SCALE ISSUES for m = 1;
        double scale = solveScale(best_X, best_transform);
        cout << "Scale : " << scale << std::endl;

        // Assign current values to the previous ones, for the next iteration
        previous_keypoints = current_keypoints;
        previous_frame = current_frame;
        previous_descriptors = current_descriptors;
    }
    return 0;
}

double solveScale(cv::Mat_<double> X, cv::Matx34d RTMatrix) {

    return 0.1;
/**    cv::Mat_<double> A (2 * X.size().width, 1, CV_32F, 0.0f);
    cv::Mat_<double> b (2 * X.size().width, 1, CV_32F, 0.0f);
    cv::Point p;

    cv::Matx13d r1 = (RTMatrix(0,0), RTMatrix(0,1), RTMatrix(0,2));
    cv::Matx13d r2 = (RTMatrix(1,0), RTMatrix(1,1), RTMatrix(1,2));
    cv::Matx13d r3 = (RTMatrix(2,0), RTMatrix(2,1), RTMatrix(2,2));
    cv::Mat<1,1,double> temp1;
    cv::Mat<1,1,double> temp2;

    cv::Matx31d point3D;

    double s_t_u = RTMatrix(0,3);
    double s_t_v = RTMatrix(1,3);
    double s_t_w = RTMatrix(1,3);

    int i = 0;
    for ( match_it = good_matches.begin(); match_it != good_matches.end(); match_it++ ) {

        p = current_keypoints[match_it->queryIdx].pt;

        point3D = cv::Matx31d(  X(0,0),
                                X(1,0),
                                X(2,0) );

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
    A = (A.t() * A).inv() * A.t();

    std::cout << matrixToString((cv::Mat)(A*b)) << std::endl;

    double s = ((cv::Mat)(A * b)).at<double>(0,0);
    std::cout << "Found scale difference: " << s << std::endl;
**/}
