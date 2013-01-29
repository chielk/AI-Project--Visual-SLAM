#ifndef CLOUD_H
#define CLOUD_H
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <time.h>
#include <vector>
#include "pcl-1.6/pcl/visualization/cloud_viewer.h"

template <class point> class Cloud
{
    public:
        void remove(int index);
        void remove(int, point &p, uchar &d, int &f);
        void remove_last(int);
        void remove_frame(int);
        void add(point, uchar, int);
        void add(std::vector<point>, std::vector<uchar>, int frame_nr);
        void get(int frame,
                typename std::vector<point>::iterator &p_begin,
                typename std::vector<point>::iterator &p_end,
                std::vector<uchar>::iterator &d_begin,
                std::vector<uchar>::iterator &d_end);
        void get_points(std::vector<point> &pts);
        void get_descriptors(std::vector<uchar> &dscs);
        void get_frames(std::vector<int> &fs);
        void show_cloud(pcl::visualization::CloudViewer &viewer, int seconds);
        Cloud();
        Cloud(std::vector<point>, std::vector<uchar>);
        ~Cloud();
    private:
        std::vector<point> points;
        std::vector<uchar> descriptors;
        std::vector<int> frames;
        void vec2cloud(std::vector<cv::Point3d> point_vector,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void vec2cloud(std::vector<cv::Point2d> point_vector,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif
