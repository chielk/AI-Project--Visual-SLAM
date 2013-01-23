#include <pcl-1.6/pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <time.h>


void mat2cloud(cv::Mat points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{ 
    cloud->width  = 1;
    cloud->height = points.cols;
    cloud->points.resize( cloud->height );

    for(int i = 0; i != cloud->height; i++) {
        cloud->points[i].x = points.at<float>(0, i);
        cloud->points[i].y = points.at<float>(1, i);
        cloud->points[i].z = points.at<float>(2, i);
    }
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    cv::Mat points = (cv::Mat_<float>(4, 10) << 
            1,2,1,2,1,2,1,2,0,0,
            1,1,2,2,1,1,2,2,0,0,
            1,1,1,1,2,2,2,2,0,0,
            1,1,1,1,1,1,1,1,1,1);
    cv::Mat shift = (cv::Mat_<float>(4, 10) << 
            1,1,1,1,1,1,1,1,0,0,
            1,1,1,1,1,1,1,1,0,0,
            1,1,1,1,1,1,1,1,0,0,
            0,0,0,0,0,0,0,0,0,0);

    mat2cloud(points, cloud);


    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);

    int increase = 1;
    int i = 0;
    while (!viewer.wasStopped()) {
        if (increase == 1 && i < 100) {
            i++;
        } else if (increase == -1 && i > 0) {
            i--;
        } else {
            increase *= -1;
        }
        points += increase*0.01*shift;
        mat2cloud(points, cloud);
        viewer.showCloud(cloud);
        usleep(30000);
    }
}
