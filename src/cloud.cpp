#include "cloud.hpp"

template <class point>
Cloud<point>::Cloud()
{
    
}

template <class point>
Cloud<point>::Cloud(std::vector<point> ps, std::vector<uchar> ds)
{
   points = ps;
   descriptors = ds;
}

template <class point>
void Cloud<point>::remove(int index, point &p, uchar &d, int &f)
{
   p = points.erase(points.begin()+index);
   d = descriptors.erase(descriptors.begin()+index);
   f = frames.erase(frames.begin()+index);
}

template <class point>
void Cloud<point>::remove(int index)
{
   points.erase(points.begin()+index);
   descriptors.erase(descriptors.begin()+index);
   frames.erase(frames.begin()+index);
}

template <class point>
void Cloud<point>::add(point p, uchar d, int f)
{
   points.push_back(p);
   descriptors.push_back(d);
   frames.push_back(f);
}

template <class point>
void Cloud<point>::add(std::vector<point> pts, std::vector<uchar> dscs, int frame_nr)
{
   points.insert(points.end(), pts.begin(), pts.end());
   descriptors.insert(descriptors.end(), dscs.begin(), dscs.end());

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
   descriptors.erase(descriptors.begin()+n);
   frames.erase(frames.begin()+n);
}

template <class point>
void Cloud<point>::get_points(std::vector<point> &pts)
{
    pts = points;
}

template <class point>
void Cloud<point>::get_descriptors(std::vector<uchar> &dscs)
{
    dscs = descriptors;
}

template <class point>
void Cloud<point>::get_frames(std::vector<int> &fs)
{
    fs = frames;
}

template <class point>
void Cloud<point>::get(int frame,
        typename std::vector<point>::iterator &p_begin,
        typename std::vector<point>::iterator &p_end,
        std::vector<uchar>::iterator &d_begin,
        std::vector<uchar>::iterator &d_end)
{
    std::vector<int>::iterator f = frames.begin();
    int i = 0;
    for(; f != frames.end(); f++, i++) {
        if (*f == frame) {
            p_begin = points.begin() + i;
            d_begin = descriptors.begin() + i;
            break;
        }
    }
    for(; f != frames.end(); f++, i++) {
        if (*f != frame) {
            p_end = points.begin() + i;
            d_end = descriptors.begin() + i;
            return;
        }
    }
    p_end = points.end();
    d_end = descriptors.end();
}

template <class point>
Cloud<point>::~Cloud()
{
}

template <class point>
void Cloud<point>::show_cloud(pcl::visualization::CloudViewer &viewer, int seconds)
{    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    vec2cloud(points, cloud);
    viewer.showCloud(cloud);
    sleep(seconds);
}

template <class point>
void Cloud<point>::vec2cloud(std::vector<cv::Point3d> point_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
void Cloud<point>::vec2cloud(std::vector<cv::Point2d> point_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
