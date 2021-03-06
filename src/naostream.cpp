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
#include "cloud.hpp"

#define VISUALIZE 1
#if VISUALIZE

#endif

#define RED cv::Scalar( 0, 0, 255 )
#define EPSILON 0.0001
#define THRESHOLD 0.05
#define VERBOSE 1

#define _BRISK 0
#define _FREAK 1
#define _ORB   2

#define _FEATURE _BRISK
#define HARTLEY_TRIANGULATION 1

enum DMMethod { 
    TS_MS, // Total Shift - Mean Shift
    MEAN_SHIFT,
    TOTAL_SHIFT
};
typedef std::vector<cv::KeyPoint> KeyPointVector;

void KeypointsToPoints(KeyPointVector keypoints, std::vector<cv::Point2d> &points) {
    for( int i = 0; i < keypoints.size(); i++ ) {
        points.push_back( keypoints[i].pt );
    }
}


class VisualOdometry
{
    InputSource *inputSource;
    cv::Matx33d K;
    cv::Mat distortionCoeffs;


    cv::Mat_<double> LinearLSTriangulation( cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2 );
    cv::Matx31d IterativeLinearLSTriangulation(cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2);
    double TestTriangulation(std::vector<cv::Point3d> &pcloud_pt3d, cv::Matx34d &P);

    double determineFundamentalMatrix(std::vector<cv::Point2d> &current_points,
                                      std::vector<cv::Point2d> &previous_points,
                                      std::vector<cv::Point2d> &previous_points_inliers,
                                      std::vector<cv::Point2d> &current_points_inliers,
                                      std::vector<cv::DMatch> &matches,
                                      cv::Matx33d &F);

    bool GetCameraMatrixHorn( cv::Matx33d &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t );
    bool DecomposeEtoRandT( cv::Matx33d &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t );

    void FindBestRandT( std::vector<cv::Point2d> &previous_points,
                        std::vector<cv::Point2d> &current_keypoints,
                        cv::Mat &R1, cv::Mat &R2, cv::Mat &t,
                        std::vector<cv::Point3d> &best_X,
                        cv::Matx34d &best_transform );

    void SolvePnPUsingRansac( std::vector<cv::DMatch> matches,
                              KeyPointVector current_keypoints,
                              std::vector<cv::Point3d> total_3D_pointcloud,
                              std::vector<cv::Point2d> &imagepoints,
                              std::vector<cv::Point3d> &objectpoints,
                              cv::Matx34d& transformationmatrix);

    void determineRollPitchYaw(double &roll, double &pitch, double &yaw, cv::Matx34d RTMatrix);
    double distanceMeasure( KeyPointVector kpv1, KeyPointVector kpv2, DMMethod method );
    double findScaleLinear(cv::Matx34d &Pcam, cv::Mat &points3d, cv::Mat &points2d);
    void TriangulatePoints(std::vector<cv::Point2d> &previous_points,
                           std::vector<cv::Point2d> &current_points,
                           cv::Matx34d &P1, cv::Matx34d &P2, std::vector<cv::Point3d> &X);

    void DetermineNewOutliers(std::vector<cv::DMatch> &matches,
                              std::vector<cv::Point2d> &current_points,
                              cv::Mat &current_descriptors,
                              std::vector<cv::Point2d> &current_outlier_points_2d,
                              cv::Mat &current_outlier_descriptors_2d);

public:
    VisualOdometry(InputSource *source);
    ~VisualOdometry();
    bool MainLoop();

    bool validConfig;
};

/**
 * Determine fundamental matrix, and while we're at it, find the inliers for both current and previous
 * points (and store them in new vectors), update matches, and calculate mean displacement.
 **/
double VisualOdometry::determineFundamentalMatrix(std::vector<cv::Point2d> &previous_points,
                                                  std::vector<cv::Point2d> &current_points,
                                                  std::vector<cv::Point2d> &previous_points_inliers,
                                                  std::vector<cv::Point2d> &current_points_inliers,
                                                  std::vector<cv::DMatch> &matches,
                                                  cv::Matx33d &F)
{
    std::vector<uchar> status( matches.size() );
    std::vector<cv::DMatch> good_matches;
    double mean_distance = 0.0;

    double minVal, maxVal;
    cv::minMaxIdx( previous_points, &minVal, &maxVal );

    // Find the fundamental matrix
    F = cv::findFundamentalMat( previous_points,
                                current_points,
                                status,
                                cv::FM_RANSAC, 0.006 * maxVal,
                                0.99 );

    // Reject outliers and calculate mean distance at the same time! Update matches as well
    for ( size_t i = 0; i < previous_points.size(); i++ ) {
        if( status[i] ) {
            current_points_inliers.push_back( current_points[i] );
            previous_points_inliers.push_back( previous_points[i] );

            // Needed for displaying inliers
            good_matches.push_back( matches[i] );

            mean_distance += cv::norm( current_points[i] - previous_points[i] );
        }
    }

    // These are matches after removing outliers
    matches = good_matches;

    // Distance calculation
    mean_distance /= (double)matches.size();
    return mean_distance;
}


bool VisualOdometry::GetCameraMatrixHorn( cv::Matx33d &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t )
{
    cv::Mat EtimesEtransposed = (cv::Mat)E * (cv::Mat)E.t();
    cv::Matx33d TTsquared;

    double trace = EtimesEtransposed.at<double>(0,0) +
                   EtimesEtransposed.at<double>(1,1) +
                   EtimesEtransposed.at<double>(2,2);

    cv::subtract( (cv::Mat) cv::Mat::eye(3,3,CV_64F) * trace * 0.5,
                  (cv::Mat)EtimesEtransposed,
                  TTsquared );

    // Three possible solutions, column with largest diagonal value is safest
    if (TTsquared(0,0) > TTsquared(1,1) && TTsquared(0,0) > TTsquared(2,2)) {
        t = (cv::Mat)TTsquared.col(0) / sqrt(TTsquared(0,0));
    } else if (TTsquared(1,1) > TTsquared(2,2)) {
        t = (cv::Mat)TTsquared.col(1) / sqrt(TTsquared(1,1));
    } else {
        t = (cv::Mat)TTsquared.col(2) / sqrt(TTsquared(2,2));
    }

    std::cout << t << std::endl;

    // Determine rotationmatrix in multiple steps
    // 1. Calculate cofactor of E
    cv::Mat e1 (E.col(0));
    cv::Mat e2 (E.col(1));
    cv::Mat e3 (E.col(2));    
    cv::Matx31d c12 = e1.cross(e2);
    cv::Matx31d c23 = e2.cross(e3);
    cv::Matx31d c31 = e3.cross(e1);

    cv::Matx33d cofactor_E( c23(0), c31(0), c12(0),
                            c23(1), c31(1), c12(1),
                            c23(2), c31(2), c12(2) );

    // 2. Construct skew symmetric of translation vector
    cv::Matx33d t_skewed ( 0,               -t.at<double>(2),  t.at<double>(1),
                           t.at<double>(2),  0              , -t.at<double>(0),
                          -t.at<double>(1),  t.at<double>(0),  0 );

    subtract( (cv::Mat)cofactor_E.t(), (cv::Mat)( (cv::Mat)t_skewed * (cv::Mat)E), R1);
    R1 /= ((cv::Mat)(t.t() * t)).at<double>(0,0);

    subtract( (cv::Mat)cofactor_E.t(), (cv::Mat)(-(cv::Mat)t_skewed * (cv::Mat)E), R2);
    R2 /= ((cv::Mat)(-t.t() * -t)).at<double>(0,0);

    return true;
}

bool VisualOdometry::DecomposeEtoRandT( cv::Matx33d &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t ) {
    //Using HZ E decomposition
    cv::SVD svd(E, cv::SVD::MODIFY_A);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabs( svd.w.at<double>( 0 ) / svd.w.at<double>( 1 ) );
    if ( singular_values_ratio > 1.0 ) {
        singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    }
    if ( singular_values_ratio < 0.7 ) {
        std::cout << "singular values are too far apart\n" << std::endl;
        std::cout << svd.w << std::endl;

        return false;
    }

    cv::Matx33d W(  0, -1,  0,	//HZ 9.513
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
cv::Mat_<double> VisualOdometry::LinearLSTriangulation( cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2 ) {
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
    cv::solve( A, B, X, cv::DECOMP_SVD );

    return X;
}


/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Matx31d VisualOdometry::IterativeLinearLSTriangulation(cv::Point3d u1, cv::Matx34d P1, cv::Point3d u2, cv::Matx34d P2	) {
    double wi1 = 1;
    double wi2 = 1;

    cv::Matx41d X;

    for ( int i = 0; i < 10; i++ ) {
        // Hartley suggests 10 iterations at most
        cv::Mat_<double> X_ = LinearLSTriangulation( u1, P1, u2, P2 );
        X = cv::Matx41d( X_(0), X_(1), X_(2), 1.0 );

        // Recalculate weights
        double p2x1 = cv::Mat_<double>( P1.row( 2 ) * X )(0);
        double p2x2 = cv::Mat_<double>( P2.row( 2 ) * X )(0);

        // Breaking point
        if ( fabs( wi1 - p2x1 ) <= EPSILON && fabs( wi2 - p2x2 ) <= EPSILON ) break;

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

bool VisualOdometry::MainLoop() {
    // Declare neccessary storage variables
    cv::Mat current_descriptors, previous_descriptors;
    KeyPointVector current_keypoints, previous_keypoints;
    std::vector<cv::DMatch> matches;
    cv::Matx41d robotPosition (0.0, 0.0, 0.0, 1.0);

    // Create brisk detector
#if _FEATURE == _BRISK
    cv::BRISK features(60, 4, 1.0f);
    features.create("BRISK");
#elif _FEATURE == _FREAK
    cv::FREAK features(60, 4, 1.0f);
    features.create("FREAK");
#elif _FEATURE == _ORB
    cv::ORB features(60, 4, 1.0f);
    features.create("ORB");
#endif
    ///
    /// ORB(int nfeatures = 500, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
    /// int firstLevel = 0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31 );
    ///

    // Get the previous frame
    Frame current_frame;
    Frame previous_frame;
    inputSource->getFrame( previous_frame );

    cv::Mat colorMat = previous_frame.img.clone();
    cv::cvtColor(colorMat, previous_frame.img, CV_BGR2GRAY);

    // Detect features for the firstm time
    features.detect( previous_frame.img, previous_keypoints );    
    features.compute( previous_frame.img, previous_keypoints, previous_descriptors );

    // Ready matcher and corresponding iterator object
    cv::FlannBasedMatcher matcher( new cv::flann::LshIndexParams( 20, 10, 2 ) );
    std::vector<cv::DMatch>::iterator match_it;

    // Use frame-to-frame initially.
    bool epnp = false;

    // Storage for 3d points and corresponding descriptors
    Cloud<cv::Point3d> cloud_3D;
    Cloud<cv::Point2d> cloud_2D;

    int frame_nr = 0;
    cv::Mat total_3D_descriptors;
    cv::Mat total_2D_descriptors;

    // Scale of initial and current image
    double current_scale, init_scale, ratio_scale;

    // Construct matrix [I|0]
    cv::Matx34d P1( 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0 );
    // Ready for construction of matrix [R|t]
    cv::Matx34d P2;

#if VISUALIZE
    pcl::visualization::CloudViewer viewer("Cloudviewer");
#endif
    while ( (char) cv::waitKey( 30 ) == -1 ) {
        // Retrieve an image
        if ( !inputSource->getFrame( current_frame ) ) {
            std::cout << "Can not read the next frame." << std::endl;
            break;
        }
        if ( !current_frame.img.data ) {
            std::cerr << "No image found." << std::endl;
            return false;
        }

        // Convert to grayscale
        cv::Mat colorMat = current_frame.img.clone();
        cv::cvtColor(colorMat, current_frame.img, CV_BGR2GRAY);

        // Detect features
        features.detect( current_frame.img, current_keypoints );

        // TODO : What if zero features found?
        if ( previous_keypoints.size() == 0 ) {
            continue;
        }

        // Find descriptors for these features
        features.compute( current_frame.img, current_keypoints, current_descriptors );

        if (epnp)
        {
            // CASE 1: SolvePnP
            cloud_3D.get_descriptors(total_3D_descriptors);
            matches.clear();
            matcher.match( current_descriptors, total_3D_descriptors, matches );

            // Determine minimum distance and derive good matches from it
            double minDist = matches[0].distance;
            for(match_it = matches.begin(); match_it != matches.end(); match_it++) {
                if(match_it->distance < minDist) minDist = match_it->distance;
            }
            std::vector<cv::DMatch> good_matches;
            for(match_it = matches.begin(); match_it != matches.end(); match_it++) {
                if(match_it->distance < 2*minDist) {
                    good_matches.push_back( *match_it );
                }
            }
# if VERBOSE
            std::cout << "Before pruning " << matches.size()
                      << " matches, after " << good_matches.size() << "." << std::endl;
#endif

            // determine correct keypoints and corresponding 3d positions
            std::vector<cv::Point3d> points_3d;
            cloud_3D.get_points(points_3d);
            std::vector<cv::Point2d> imagepoints;
            std::vector<cv::Point3d> objectpoints;
            SolvePnPUsingRansac(good_matches, current_keypoints, points_3d, imagepoints, objectpoints, P2);

            // Print out [R|T]
            std::cout << P2 << std::endl;

            //////////////////////////////////
            // Triangulate any (yet) unknown points
            cloud_2D.get_descriptors(total_2D_descriptors);
            if(!total_2D_descriptors.empty()) {
                matches.clear();
                matcher.match(current_descriptors, total_2D_descriptors, matches);
                std::vector<cv::Point2d> matching_2D_points, current_points, total_2D_points;
                cloud_2D.get_points(total_2D_points);

                for ( match_it = matches.begin(); match_it != matches.begin() + current_descriptors.rows; match_it++ ) {
                    current_points.push_back( current_keypoints[match_it->queryIdx].pt );
                    matching_2D_points.push_back( total_2D_points[match_it->trainIdx] );
                }

                cv::Matx33d fundamental;
                std::vector<cv::Point2d> previous_points_inliers, current_points_inliers;
                double mean_distance = determineFundamentalMatrix(matching_2D_points,
                                                                  current_points,
                                                                  previous_points_inliers,
                                                                  current_points_inliers,
                                                                  matches,
                                                                  fundamental);

                std::vector<cv::Point2d> current_outlier_points_2d;
                cv::Mat current_outlier_descriptors_2d;

                DetermineNewOutliers(matches,
                                     current_points,
                                     current_descriptors,
                                     current_outlier_points_2d,
                                     current_outlier_descriptors_2d);

                cloud_2D.add(current_outlier_points_2d, current_outlier_descriptors_2d, frame_nr);

                // If displacement is not sufficiently large, skip this image.
                if (mean_distance < THRESHOLD)
                {
                    if (mean_distance < 1)
                    {
                        // what the fuck just happened
                        std::cout << "wut 0 mean_distance wut" << std::endl;
                    }
        #if VERBOSE
                    std::cout << "Displacement not sufficiently large, skipping frame." << std::endl;
        #endif
                    continue;
                }

                std::vector<cv::Point3d> X;
                TriangulatePoints(matching_2D_points,
                                  current_points,
                                  P1,
                                  P2,
                                  X);

                //cloud_3D.add(X, , frame_nr);

           // Add to all_descriptors and 3d point cloud
            } else {
                // If no 2d points available, add all current points to the cloud.
                std::vector<cv::Point2d> current_points;
                KeypointsToPoints(current_keypoints, current_points);
                cloud_2D.add(current_points, current_descriptors, frame_nr);
            }


        } else {

            // CASE 0: frame-to-frame

            // Match descriptor vectors using FLANN matcher
            matcher.match( current_descriptors, previous_descriptors, matches );

            // Calculation of centroid by looping over matches
            cv::Point2d current_centroid(0,0);
            cv::Point2d previous_centroid(0,0);
            std::vector<cv::Point2d> current_points_normalized, previous_points_normalized;
            cv::Point2d cp;
            cv::Point2d pp;

            for ( match_it = matches.begin(); match_it != matches.begin() + current_descriptors.rows; match_it++ ) {
                cp = current_keypoints[match_it->queryIdx].pt;
                pp = previous_keypoints[match_it->trainIdx].pt;

                current_centroid.x += cp.x;
                current_centroid.y += cp.y;
                current_points_normalized.push_back( cp );

                previous_centroid.x += pp.x;
                previous_centroid.y += pp.y;
                previous_points_normalized.push_back( pp );
            }

            // Normalize the centroids
            int matchesSize = matches.size();
            current_centroid.x /= matchesSize;
            current_centroid.y /= matchesSize;
            previous_centroid.x /= matchesSize;
            previous_centroid.y /= matchesSize;

            double current_scaling = 0;
            double previous_scaling = 0;

            // Translate points to have (0,0) as centroid
            for ( size_t i = 0; i < matches.size(); i++ ) {
                current_points_normalized[i] -= current_centroid;
                previous_points_normalized[i] -= previous_centroid;

                current_scaling += cv::norm( current_points_normalized[i] );
                previous_scaling += cv::norm( previous_points_normalized[i] );
            }

            // Enforce mean distance sqrt( 2 ) from origin (0,0)
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

            // Scale points
            for ( size_t i = 0; i < matches.size(); i++ ) {
                previous_points_normalized[i] *= previous_scaling;
                current_points_normalized[i]  *= current_scaling;
            }

            // Find the fundamental matrix and reject outliers and calc distance between matches
            cv::Matx33d F;
            std::vector<cv::Point2d> current_points_normalized_inliers, previous_points_normalized_inliers;

            double mean_distance = determineFundamentalMatrix(previous_points_normalized,
                                                              current_points_normalized,
                                                              previous_points_normalized_inliers,
                                                              current_points_normalized_inliers,
                                                              matches,
                                                              F);



            // Add outliers to the 2d cloud of unused features
            std::vector<cv::Point2d> all_points_current, all_points_previous;
            KeypointsToPoints( current_keypoints, all_points_current );
            KeypointsToPoints( previous_keypoints, all_points_previous );

            std::vector<cv::Point2d> current_outlier_points_2d;
            cv::Mat current_outlier_descriptors_2d;

            DetermineNewOutliers(matches,
                                 all_points_current,
                                 current_descriptors,
                                 current_outlier_points_2d,
                                 current_outlier_descriptors_2d);

            cloud_2D.add(current_outlier_points_2d, current_outlier_descriptors_2d, frame_nr);

            // Scale up again
            F = current_T.t() * F * previous_T;

    #if VERBOSE
            std::cout << "Matches before pruning: " << matchesSize << ". " <<
                         "Matches after: " << matches.size() << "\n" <<
                         "Mean displacement: " << mean_distance << std::endl;
    #endif

            // Draw only inliers
            cv::Mat img_matches;
            cv::drawMatches(
                current_frame.img, current_keypoints, previous_frame.img, previous_keypoints,
                matches, img_matches, cv::Scalar::all( -1 ), cv::Scalar::all( -1 ),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
                );

            imshow( "Good Matches", img_matches );
            imwrite("some.png", img_matches);

            // If displacement is not sufficiently large, skip this image.
            std::cout << mean_distance << std::endl;
            if (mean_distance < THRESHOLD)
            {
                if (mean_distance < 0.00001)
                {
                    // what the fuck just happened
                    std::cout << "wut 0 mean_distance wut" << std::endl;
                }
    #if VERBOSE
                std::cout << "Displacement not sufficiently large, skipping frame." << std::endl;
    #endif
                continue;
            }

            // Compute essential matrix
            cv::Matx33d E (cv::Mat(K.t() * F * K));

            // Estimation of projection matrix
            cv::Mat R1, R2, t;
            if( !GetCameraMatrixHorn( E, R1, R2, t ) ) {
                return false;
            }

            // Check correctness(!(cv::Mat(visualOdometry->K).empty()))
            if ( cv::determinant(R1) < 0 ) R1 = -R1;
            if ( cv::determinant(R2) < 0 ) R2 = -R2;

            std::vector<cv::Point3d> best_X;
            cv::Matx34d best_transform;

            std::vector<cv::Point2d> cpoints, ppoints;
            for ( size_t m = 0; m < matches.size(); m++ ) {
                cpoints.push_back( current_keypoints[matches[m].queryIdx].pt );
                ppoints.push_back( previous_keypoints[matches[m].trainIdx].pt );
            }

            FindBestRandT(ppoints, cpoints, R1, R2, t, best_X, best_transform);

#if VERBOSE
            std::cout << "Best found transformation\n" << best_transform << "\n" << std::endl;
            for ( size_t x  = 0; x < best_X.size(); x++ ) {
                std::cout << ppoints[x] << "\t" << cpoints[x] << "\t" << best_X[x] << std::endl;
            }
#endif
#if VISUALIZE
            // Display found points
            cloud_3D.show_cloud(viewer, 3);
#endif

            /**
             *  Input  - Pcam  -> (3x4) Camera position
             *         - X3D   -> (3xn) 3D points
             *         - Q     -> (2xn) Image points
             *         - K     -> (3x3) Camera calibration
             *
             *  Output - scale -> (1x1) Scaling factor
             *         - Pcam  -> (3x4) Scaled camera matrix
             **/
            // SOLVE THEM SCALE ISSUES for m = 1;
            cv::Mat imagepoints (2, best_X.size(), CV_64F);
            cv::Mat objectpoints(3, best_X.size(), CV_64F);

            for ( size_t n = 0; n < best_X.size(); n++) {
                imagepoints.at<double>(0, n) = current_keypoints[matches[n].queryIdx].pt.x;
                imagepoints.at<double>(1, n) = current_keypoints[matches[n].queryIdx].pt.y;

                cv::Point3d pt = best_X[n];
                objectpoints.at<double>(0,n) = pt.x;
                objectpoints.at<double>(1,n) = pt.y;
                objectpoints.at<double>(2,n) = pt.z;
            }
            
            std::cout << "Finding scale..." << std::endl;

            double norm_t = cv::norm(best_transform.col(3));
            best_transform(0,3) /= norm_t;
            best_transform(1,3) /= norm_t;
            best_transform(2,3) /= norm_t;

            std::cout << best_transform << std::endl;

            if(frame_nr == 0) {
                init_scale = findScaleLinear(best_transform,
                                            objectpoints,
                                            imagepoints);
                std::cout << "Scale First Frame: " << init_scale << std::endl;
            } else {
                current_scale = findScaleLinear(best_transform,
                                                objectpoints,
                                                imagepoints);

                std::cout << "Scale First Frame: "   << init_scale << std::endl;
                std::cout << "Scale Current Frame: " << current_scale << std::endl;

                std::cout << "Rescale points..." << std::endl;
                // You want to scale the points towards your init scale.
                ratio_scale = init_scale / current_scale;
                best_transform(0,3) *= ratio_scale;
                best_transform(1,3) *= ratio_scale;
                best_transform(2,3) *= ratio_scale;

                // Clear current best_X and obtain it with new scaling
                best_X.clear();

                cv::Matx34d P1( 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0 );

                TriangulatePoints(ppoints, cpoints, P1, best_transform, best_X);
#if VERBOSE
                std::cout << "Scaled found transformation\n" << best_transform << "\n" << std::endl;
#endif
            }

#if VERBOSE
            std::cout << "Found points: " << std::endl;
            for ( size_t x = 0; x < best_X.size(); x++ ) {
                std::cout << best_X[x] << std::endl;
            }
#endif

            // Update total points/cloud
            std::cout << "Storing points" << std::endl;
            total_3D_descriptors = cv::Mat( matches.size(), current_descriptors.size().width, current_descriptors.type());
            for ( size_t matchnr = 0; matchnr < matches.size(); matchnr++) {
                 total_3D_descriptors.row(matchnr) = current_descriptors.row(matchnr);
            }
            cloud_3D.add(best_X, total_3D_descriptors, frame_nr);
            
            // TODO BE SMART

            //cv::Matx44d transformationMatrix( best_transform(0,0),best_transform(0,1),best_transform(0,2),best_transform(0,3),
            //                                  best_transform(1,0),best_transform(1,1),best_transform(1,2),best_transform(1,3),
            //                                  best_transform(2,0),best_transform(2,1),best_transform(2,2),best_transform(2,3),
            //                                  0, 0, 0, 1 );

            cv::Matx44d transformationMatrix;
            cv::vconcat( best_transform, cv::Matx14d(0, 0, 0, 1), transformationMatrix );

            robotPosition = transformationMatrix * robotPosition;
            robotPosition(0,0) /= robotPosition(3,0);
            robotPosition(1,0) /= robotPosition(3,0);
            robotPosition(2,0) /= robotPosition(3,0);
            robotPosition(3,0) /= robotPosition(3,0);

            std::cout << "Position: " << robotPosition.t() << std::endl;

            double roll, pitch, yaw;
            determineRollPitchYaw(roll, pitch, yaw, best_transform);
            //std::cout << "roll" << roll << "\n"
            //          << "pitch" << pitch << "\n"
            //          << "yaw" << yaw << std::endl;

            // Assign current values to the previous ones, for the next iteration
            previous_keypoints = current_keypoints;
            previous_frame = current_frame;
            previous_descriptors = current_descriptors;


            //epnp = true;
        }
        frame_nr++;
    }
    // Main loop successful.
    return true;
}

double VisualOdometry::TestTriangulation(std::vector<cv::Point3d> &pcloud_pt3d, cv::Matx34d &P) {
    std::vector<uchar> status;
    std::vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

    cv::Matx44d P4x4 = cv::Matx44d::eye();

    for(int i = 0; i < 12; i++) {
        P4x4.val[i] = P.val[i];
    }

    cv::perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

    status.resize(pcloud_pt3d.size(),0);
    for (int i=0; i<pcloud_pt3d.size(); i++) {
        status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
    }
    int count = cv::countNonZero(status);

    double percentage = ((double)count / (double)pcloud_pt3d.size());
//#if VERBOSE
    std::cout << count << "/" << pcloud_pt3d.size() << " = " << percentage*100.0 << "% are in front of camera" << std::endl;
//#endif
    return percentage;

}

/**
 *  Input  - Pcam  -> (3x4) Camera position
 *         - X3D   -> (3xn) 3D points
 *         - Q     -> (2xn) Image points
 *         - K     -> (3x3) Camera calibration
 *
 *  Output - scale -> (1x1) Scaling factor
 *         - Pcam  -> (3x4) Scaled camera matrix
 **/
double VisualOdometry::findScaleLinear(cv::Matx34d &Pcam,
                                       cv::Mat &points3d,
                                       cv::Mat &points2d) {

    // Make 2d points homogeneous
    cv::Mat points2d_homogeneous;
    cv::vconcat(points2d,
                cv::Mat::ones(1, points2d.size().width, points2d.type()),
                points2d_homogeneous);

    // Make 3d points homogeneous
    cv::Mat points3d_homogeneous;
    cv::vconcat(points3d,
                cv::Mat::ones(1, points3d.size().width, points3d.type()),
                points3d_homogeneous);
    //std::cout << "Homogeneous points" << std::endl;
    //std::cout << points2d_homogeneous << std::endl;

    // Put 2d points in space K-1 x Q, form 3 x n matrix Qw
    cv::Mat Qw = ((cv::Mat)K.inv()) * points2d_homogeneous;

    // Project 3D points in image:
    // 3xn = 3x4 * 4xn
    //cv::Mat Pw = ((cv::Mat)Pcam) * points3d_homogeneous;
    //Pw = ((cv::Mat)K) * Pw;

    // Divide by homogeneous thingemajizz
    //for ( int i = 0; i < Pw.size().width; i++) {
    //    Pw.at<double>(0,i) = Pw.at<double>(0,i) / Pw.at<double>(2,i);
    //    Pw.at<double>(1,i) = Pw.at<double>(1,i) / Pw.at<double>(2,i);
    //    Pw.at<double>(2,i) = 1.0;
    //}

    double scale;
    // Build matrix A and vector b
    cv::Mat_<double> A ( 2 * points2d.size().width, 1 );
    cv::Mat_<double> b ( 2 * points2d.size().width, 1 );

    cv::Matx13d r1( Pcam(0,0), Pcam(0,1), Pcam(0,2) );
    cv::Matx13d r2( Pcam(1,0), Pcam(1,1), Pcam(1,2) );
    cv::Matx13d r3( Pcam(2,0), Pcam(2,1), Pcam(2,2) );

    cv::Matx23d r12;
    cv::vconcat(r1,r2,r12);

    cv::Matx21d tu (Pcam(0,3), Pcam(1,3));

    cv::Matx21d temp1, temp2;
    for ( int i = 0; i < points3d.size().width; i++ ) {
        //temp1 = ( Pcam(1:2,1:3) * X3D(1:3,i)  -
        //         (Pcam(3,1:3) * X3D(1:3,i)) * Qw(1:2,i));
        cv::Matx31d pointX ( points3d.at<double>(0,i),
                             points3d.at<double>(1,i),
                             points3d.at<double>(2,i) );
        cv::Matx21d pointx ( Qw.at<double>(0,i),
                             Qw.at<double>(1,i) );

        cv::subtract( ((cv::Mat)r12) * (cv::Mat)pointX,
                      (cv::Mat)pointx * ((cv::Mat)((cv::Mat)r3 * (cv::Mat)pointX)),
                      temp1);

        //temp2 = Pcam(3,4) * Qw(1:2,i) - Pcam(1:2,4);
        cv::subtract( Pcam(2,3) * (cv::Mat)pointx, (cv::Mat)tu, temp2);

        A.at<double>(i*2,   0) = temp2(0);
        A.at<double>(i*2+1, 0) = temp2(1);
        b.at<double>(i*2,   0) = temp1(0);
        b.at<double>(i*2+1, 0) = temp1(1);
    }

    cv::Mat scalemat = ((A.t() * b) / (A.t() * A));
    scale = scalemat.at<double>(0, 0);


    // METHOD 4
    double scale2;
    cv::Mat_<double> A2 ( points2d.size().width, 1 );
    cv::Mat_<double> b2 ( points2d.size().width, 1 );

    for ( int i = 0; i < points3d.size().width; i++ ) {
        cv::Matx31d pointX ( points3d.at<double>(0,i),
                             points3d.at<double>(1,i),
                             points3d.at<double>(2,i) );
        cv::Point2d pointx ( Qw.at<double>(0,i),
                             Qw.at<double>(1,i) );

        //                  Pcam(2,4) * ( Qw(1,i)            / Qw(2,i))            - Pcam(1,4);
        A2.at<double>(i, 0) = Pcam(1,3) * (pointx.x / pointx.y) - Pcam(0,3);

        //                  (Pcam(1,1:3)-Pcam(2,1:3) *
        //                  (Qw(1,i) / Qw(2,i))) * X3D(1:3,i)
        cv::Mat temp = ((cv::Mat) r1 - ((cv::Mat) r2 * (pointx.x / pointx.y))) * (cv::Mat) pointX;
        b2.at<double>(i, 0) = temp.at<double>(0,0);
    }
    scale2 = (double)((cv::Mat)((cv::Mat(A.t() * A)) * A.t() *b)).at<double>(0,0);

    scalemat = ((A2.t() * b2) / (A2.t() * A2));
    scale2 = scalemat.at<double>(0, 0);

    std::cout << Pcam << std::endl;
    std::cout << "Scale method 3: " << scale << std::endl;
    std::cout << "Scale method 4: " << scale2 << std::endl;

    return scale;
}

void VisualOdometry::determineRollPitchYaw(double &roll, double &pitch, double &yaw, cv::Matx34d RTMatrix)
{
    // Order of rotation must be roll pitch yaw for this to work
    roll = atan2(RTMatrix(1,0), RTMatrix(0,0));
    pitch = atan2(-RTMatrix(2,0), sqrt( pow(RTMatrix(2,1), 2) + pow(RTMatrix(2,2), 2) ) );
    yaw = atan2(RTMatrix(2,1), RTMatrix(2,2));
}

VisualOdometry::VisualOdometry(InputSource *source){

    this->inputSource = source;

    // Load calibrationmatrix K (and distortioncoefficients while we're at it).
    this->validConfig = loadSettings( K, distortionCoeffs);
}

VisualOdometry::~VisualOdometry(){
    delete this->inputSource;
}

double VisualOdometry::distanceMeasure( KeyPointVector kpv1, KeyPointVector kpv2, DMMethod method = MEAN_SHIFT ) {
    assert( kpv1.size() == kpv2.size() );
        cv::Point2d mean_shift( 0, 0 );
        cv::Point2d total_shift( 0, 0 );
        cv::Point2d temp_shift;
        for ( int i = 0; i < kpv1.size(); i++ ) {
            temp_shift = kpv1[i].pt - kpv2[i].pt;
            total_shift += cv::Point2d( abs( temp_shift.x ), abs( temp_shift.y ) );
            mean_shift += temp_shift;
        }
    switch ( method ) {
    case TOTAL_SHIFT:
        return sqrt( total_shift.ddot( total_shift ) );
    case MEAN_SHIFT:
        return sqrt( mean_shift.ddot( mean_shift ) );
    case TS_MS:
        temp_shift = total_shift - mean_shift; //cv::Point2d (mean_shift.x / total_shift.x, mean_shift.y / total_shift.y );
        return sqrt( temp_shift.ddot( temp_shift ) );
    default:
        return 0.0;
    }
}

int main( int argc, char* argv[] ) {
    if ( argc < 3 ) {
        std::cerr << "Usage" << argv[0] << " '(-n robotIp|-f folderName)'" << std::endl;
        return 1;
    }

    VisualOdometry *visualOdometry;
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

    visualOdometry = new VisualOdometry( inputSource );
    if (visualOdometry->validConfig)
    {
        visualOdometry->MainLoop();
    }
}

void VisualOdometry::SolvePnPUsingRansac( std::vector<cv::DMatch> matches,
                                          KeyPointVector current_keypoints,
                                          std::vector<cv::Point3d> total_3D_pointcloud,
                                          std::vector<cv::Point2d> &imagepoints,
                                          std::vector<cv::Point3d> &objectpoints,
                                          cv::Matx34d& transformationmatrix) {
    cv::Point2d ip;
    cv::Point3d op;
    cv::Point3f op_float;
    cv::Point2f ip_float;
    std::vector<cv::Point3f> objectpoints_float;
    std::vector<cv::Point2f> imagepoints_float;

    for (int i = 0; i < matches.size(); i++) {
        ip = current_keypoints[matches[i].queryIdx].pt;
        ip_float = cv::Point2f( ip.x, ip.y );
        op = total_3D_pointcloud[matches[i].trainIdx];
        op_float = cv::Point3f( op.x, op.y, op.z );

        imagepoints.push_back(ip);
        imagepoints_float.push_back(ip_float);
        objectpoints.push_back(op);
        objectpoints_float.push_back(op_float);
    }

    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(objectpoints_float, imagepoints_float, K, distortionCoeffs, rvec, tvec,
                       false, 100, 8.0, 100, inliers);


    // Construct matrix [R|t]
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    std::cout << R << std::endl;
    cv::hconcat(R, tvec, transformationmatrix);
}

/**
  * Find best transformation matrix best_transform, with corresponding triangulationpoints best_X, based
  * on candidates R1,R2 and t, and the points in the image that were not rejected by RANSAC.
  */
void VisualOdometry::FindBestRandT(std::vector<cv::Point2d> &previous_points,
                                   std::vector<cv::Point2d> &current_points,
                                   cv::Mat &R1, cv::Mat &R2, cv::Mat &t,
                                   std::vector<cv::Point3d> &best_X,
                                   cv::Matx34d &best_transform)
{
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
    std::vector<cv::Point3d> X;

    double best_percentage = 0.0;
    double percentage;

    // Loop over possible candidates
    for ( int i = 0 ; i < 4; i++ ) {

        X.clear();
        P2 = possible_projections[i];

        TriangulatePoints(previous_points,
                          current_points,
                          P1,
                          P2,
                          X);

        percentage = TestTriangulation(X, P2);

        if ( percentage > best_percentage ) {
            // update best values-so-far
            best_percentage = percentage;
            best_X = X;
            best_transform = cv::Mat(P2).clone();
        }
    }
}

void VisualOdometry::TriangulatePoints(std::vector<cv::Point2d> &previous_points,
    std::vector<cv::Point2d> &current_points,
    cv::Matx34d &P1, cv::Matx34d &P2, std::vector<cv::Point3d> &X)
{
    cv::Mat Kinv = (cv::Mat)K.inv();
#if HARTLEY_TRIANGULATION
    // TODO replace by iterator?
    for ( size_t m = 0; m < previous_points.size(); m++ ) {
        cv::Point3d current_point_homogeneous = cv::Point3d( current_points[m].x,
                                                             current_points[m].y,
                                                             1 );
        cv::Point3d previous_point_homogeneous( previous_points[m].x,
                                                previous_points[m].y,
                                                1 );

        cv::Matx31d k_current_point ( (cv::Mat)(Kinv * cv::Mat( current_point_homogeneous )));
        current_point_homogeneous.x = k_current_point(0);
        current_point_homogeneous.y = k_current_point(1);
        current_point_homogeneous.z = k_current_point(2);

        cv::Matx31d k_previous_point( (cv::Mat)(Kinv * cv::Mat( previous_point_homogeneous )));
        previous_point_homogeneous.x = k_previous_point(0);
        previous_point_homogeneous.y = k_previous_point(1);
        previous_point_homogeneous.z = k_previous_point(2);

        cv::Matx31d X_a = IterativeLinearLSTriangulation(
                previous_point_homogeneous,	P1,
                current_point_homogeneous, P2 );

        X.push_back( cv::Point3d( X_a(0), X_a(1), X_a(2) ));
    }
#else
    // Use opencvs triangulation method
    cv::Mat X_4d(4, cpoints.size(), CV_64F);
    cv::triangulatePoints(P1, P2, previous_points, current_points, X_4d);
    for ( size_t m = 0; cpoints.size(); m++ ) {
        X.push_back( cv::Point3d( X_4d.at<double>(0, m),
                                  X_4d.at<double>(1, m),
                                  X_4d.at<double>(2, m) ));
    }
#endif
}

void VisualOdometry::DetermineNewOutliers(std::vector<cv::DMatch> &matches,
                                          std::vector<cv::Point2d> &current_points,
                                          cv::Mat &current_descriptors,
                                          std::vector<cv::Point2d> &current_outlier_points_2d,
                                          cv::Mat &current_outlier_descriptors_2d)
{
    bool outlier;

    // Loop over all points in previous to find ones that matches does not refer to.
    for (int i = 0 ; i < current_points.size(); i++) {
        outlier = true;
        for(int j = 0; j < matches.size(); j++) {
            if (matches[j].queryIdx == i)  {
                outlier = false;
                break;
            }
        }
        if (outlier) {
            current_outlier_points_2d.push_back( current_points[i] );
            if(current_outlier_descriptors_2d.empty()) {
                current_outlier_descriptors_2d = current_descriptors.row(i);
            } else {
                cv::vconcat(current_outlier_descriptors_2d, current_descriptors.row(i), current_outlier_descriptors_2d );
            }
        }
    }
}
