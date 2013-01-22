#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

typedef std::vector<cv::KeyPoint> KeyPointVector;

/*
    Datastructure:
    ---
    typedef struct
    {
        cv::Mat<cv::Point2f> 2DPoints;
        cv::Mat 2DDescriptors;
        cv::Mat Timestamp + Saliency;
    } 2D_info;  

    typedef struct
    {
    - cv::Mat<cv::Point3f> 3DPoints
    - cv::Mat 3DDescriptors
    - std::vector<int>
    } 3D_info;
    ---
    -> Filter functions for 2D and 3D points to take out specific points
*/

// Perform filtering with a certain function
template<class pt>
void filtered( const cv::Mat points, bool (*function)(pt), std::vector<int> &indices )
{
    cv::MatConstIterator_<pt> it = points.begin<pt>();
    cv::MatConstIterator_<pt> end = points.end<pt>();
    int i = 0;
    for( ; it != end; it++ ) {
        if(function((*it))) {
            indices.push_back(i);
        }
        i++;
    }
}

template<class pt>
void filter( const cv::Mat points, const cv::Mat descriptors, bool (*function)(pt), cv::Mat &new_points, cv::Mat &new_descriptors  )
{
    std::vector<pt> r_pts;
    std::vector<uchar> r_dsc;
    cv::MatConstIterator_<uchar> dsc_it = descriptors.begin<uchar>();
    cv::MatConstIterator_<pt> pts_it = points.begin<pt>();
    cv::MatConstIterator_<pt> end = points.end<pt>();
    for( ; pts_it != end; pts_it++, dsc_it++ ) {
        if(function((*pts_it))) {
            r_pts.push_back(*pts_it);
            r_dsc.push_back(*dsc_it);
        }
    }
    new_points = cv::Mat(r_pts);
    new_descriptors = cv::Mat(r_dsc);
}
// Function for filtering 2D points
bool f2(cv::Point2f p)
{
    /*
    if (p.y > 1) {
        return true;
    }*/
    return false;
}

// Function for filtering 3D points
bool f3(cv::Point3f p)
{
    if (p.y > 1) {
        return true;
    }
    return false;
}


int main() {
    cv::Mat image;
    image = cv::imread( "test.jpg", 1 );

    cv::Mat descriptors;
    KeyPointVector keypoints;

    cv::BRISK brisk(60, 4, 1.0f);
    brisk.create("BRISK");

    brisk.detect( image, keypoints );    
    brisk.compute( image, keypoints, descriptors );

    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20,10,2));
    std::vector<cv::DMatch> matches;

    matcher.match( descriptors, descriptors, matches );

    std::vector<cv::DMatch>::iterator matchit;
    for(matchit = matches.begin(); matchit != matches.end(); matchit++)
    {
        //std::cout << matchit->trainIdx << " - " << matchit->queryIdx << std::endl;
    }

    cv::Mat m3d = (cv::Mat_<cv::Point3f>(3, 1) << cv::Point3f(0, 1, 2), cv::Point3f(1, 2, 3), cv::Point3f(2, 3, 4));
    std::vector<int> indices3;
    cv::Mat desc(std::vector<uchar>(3, 'a'));
    filtered(m3d, &f3, indices3);
    cv::Mat new_m3d, new_desc;
    std::cout << "[" << new_desc.rows << " " << new_desc.cols << "]" << std::endl;
    filter(m3d, desc, &f3, new_m3d, new_desc);
    std::cout << "[" << new_desc.rows << " " << new_desc.cols << "]" << std::endl;
    //cv::Mat m2d = (cv::Mat_<cv::Point2f>(3, 1) << cv::Point2f(0, 10), cv::Point2f(1, 0), cv::Point2f(2, 0));
    //std::vector<int> indices2;
    //filtered(m2d, &f2, indices2);

//    std::vector<int>::iterator it;
//    for(it = indices3.begin(); it != indices3.end(); it++) {
//        std::cout << *it << " ";
//    }
//    std::cout << std::endl;

//    for(it = indices2.begin(); it != indices2.end(); it++) {
//        std::cout << *it << " ";
//    }
    std::cout << std::endl;

    cv::namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
    cv::imshow( "Display Image", image );

    //while( cv::waitKey(0) == -1 )
    //{
    //}
}
