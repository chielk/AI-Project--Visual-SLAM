#ifndef CLOUD_H
#define CLOUD_H
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>
#include <time.h>
#include <vector>
#include "pcl-1.6/pcl/visualization/cloud_viewer.h"

template <class point> class Cloud
{
    public:
        void remove(int index);
        void remove(int, point &p, cv::KeyPoint &kp, uchar &d, int &f);
        void remove_last(int);
        void remove_frame(int);
        void add(std::vector<point>, std::vector<cv::KeyPoint>, cv::Mat, int frame_nr);
        void replace(std::vector<point>, std::vector<cv::KeyPoint> kpts, cv::Mat, int frame_nr);
        //void get(int frame,
        //        typename std::vector<point>::iterator &p_begin,
        //        typename std::vector<point>::iterator &p_end,
        //        cv::Mat &d);
        void get_points(std::vector<point> &pts);
        void get_keypoints(std::vector<cv::KeyPoint> &kpts);
        void get_descriptors(cv::Mat &dscs);
        void get_frames(std::vector<int> &fs);
        void show_cloud(pcl::visualization::CloudViewer &viewer, int seconds);
        Cloud();
        Cloud(std::vector<point> pt, std::vector<cv::KeyPoint> kp, cv::Mat dscr);
        ~Cloud();
    private:
        std::vector<point> points;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        std::vector<int> frames;
        void vec2cloud(std::vector<cv::Point3d> point_vector,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void vec2cloud(std::vector<cv::Point2d> point_vector,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

   template <class point>
Cloud<point>::Cloud()
{

}

   template <class point>
Cloud<point>::Cloud(std::vector<point> ps, std::vector<cv::KeyPoint> kp, cv::Mat ds)
{
   points = ps;
   keypoints = kp;
   descriptors = ds;
}

   template <class point>
void Cloud<point>::remove(int index, point &p, cv::KeyPoint &kp, uchar &d, int &f)
{
   p = points.erase(points.begin()+index);
   kp = keypoints.erase(keypoints.begin()+index);
   cv::hconcat(descriptors.colRange(0, index),
               descriptors.colRange(index+1, descriptors.rows),
               descriptors);
   f = frames.erase(frames.begin()+index);
}

template <class point>
void Cloud<point>::remove(int index)
{
   points.erase(points.begin()+index);
   keypoints.erase(keypoints.begin()+index);
   //descriptors.erase(descriptors.begin()+index);
   cv::hconcat(descriptors.colRange(0, index),
               descriptors.colRange(index+1, descriptors.rows),
               descriptors);
   frames.erase(frames.begin()+index);
}

template <class point>
void Cloud<point>::replace(std::vector<point> pts,
                           std::vector<cv::KeyPoint> kpts,
                           cv::Mat dscs,
                           int frame_nr)
{
   points = pts;
   dscs.copyTo(descriptors);
   keypoints = kpts;
   std::vector<int> fs(pts.size(), frame_nr);
   frames = fs;
}

template <class point>
void Cloud<point>::add(std::vector<point> pts, std::vector<cv::KeyPoint>, cv::Mat dscs, int frame_nr)
{
   points.insert(points.end(), pts.begin(), pts.end());
   //descriptors.insert(descriptors.end(), dscs.begin(), dscs.end());
   if(descriptors.empty()) {
       dscs.copyTo(descriptors);
   } else {
       cv::vconcat(descriptors, dscs, descriptors);
   }

   std::vector<int> fs(pts.size(), frame_nr);
   frames.insert(frames.end(), fs.begin(), fs.end());
}

template <class point>
void Cloud<point>::remove_last(int n)
{
   if (n <= 0) {
      return;
   }
   points.erase(points.begin()+n);
   keypoints.erase(keypoints.begin()+n);
   descriptors = descriptors(cv::Range(0, 0), cv::Range(n, 1));
   frames.erase(frames.begin()+n);
}

template <class point>
void Cloud<point>::get_points(std::vector<point> &pts)
{
    pts = points;
}

template <class point>
void Cloud<point>::get_descriptors(cv::Mat &dscs)
{
    dscs = descriptors;
}

template <class point>
void Cloud<point>::get_frames(std::vector<int> &fs)
{
    fs = frames;
}

template <class point>
void Cloud<point>::get_keypoints(std::vector<cv::KeyPoint>& kpts)
{
    kpts = keypoints;
}

/**
template <class point>
void Cloud<point>::get(int frame,
        typename std::vector<point>::iterator &p_begin,
        typename std::vector<point>::iterator &p_end,
        cv::Mat &d)
{
    std::vector<int>::iterator f = frames.begin();
    int i = 0;
    cv::Range d_begin, d_end;
    for(; f != frames.end(); f++, i++) {
        if (*f == frame) {
            p_begin = points.begin() + i;
            //d_begin = descriptors.begin() + i;
            d_begin = cv::Range(i, 0);
            break;
        }
    }
    for(; f != frames.end(); f++, i++) {
        if (*f != frame) {
            p_end = points.begin() + i;
            //d_end = descriptors.begin() + i;
            d_end = cv::Range(i, 1);
            return;
        }
    }
    p_end = points.end();
    //d_end = descriptors.end();
    d = descriptors(d_begin, d_end);
}
**/

template <class point>
Cloud<point>::~Cloud()
{
}

template <class point>
void Cloud<point>::show_cloud(pcl::visualization::CloudViewer &viewer,
      int seconds)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
       (new pcl::PointCloud<pcl::PointXYZ>);
    vec2cloud(points, cloud);
    viewer.showCloud(cloud);
    sleep(seconds);
}

template <class point>
void Cloud<point>::vec2cloud(std::vector<cv::Point3d> point_vector,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->width  = 1;
    cloud->height = point_vector.size();
    cloud->points.resize( cloud->height );

    cv::Point3d pt;
    for(int i = 0; i != cloud->height; i++) {
        pt = point_vector[i];

        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
        cloud->points[i].z = pt.z;
    }
}

template <class point>
void Cloud<point>::vec2cloud(std::vector<cv::Point2d> point_vector,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud->width  = 1;
    cloud->height = point_vector.size();
    cloud->points.resize( cloud->height );

    cv::Point2d pt;
    for(int i = 0; i != cloud->height; i++) {
        pt = point_vector[i];

        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
    }
}

#endif
