#include "cloud.cpp"
#include <iostream>

int main ()
{
    Cloud<cv::Point3d> info_3D;

    std::vector<cv::Point3d> pts;
    std::vector<uchar> dscs;
    std::vector<int> fs;
    cv::Point3d pt(1, 2, 3);
    uchar dsc = 123;
    int frame = 3;

    pts.push_back(pt);
    dscs.push_back(dsc);
    fs.push_back(frame);

    // Push 3 times a vector of points to the structure
    info_3D.add(pts, dscs, 1);
    info_3D.add(pts, dscs, 2);
    info_3D.add(pts, dscs, 3);

    // Create answer structures
    std::vector<int> answer1;
    std::vector<uchar> answer2;
    std::vector<cv::Point3d> answer3;
    info_3D.get_frames(answer1);
    info_3D.get_descriptors(answer2);
    info_3D.get_points(answer3);
    // Print last structure
    //std::cout << answer1.front() << std::endl;
    //std::cout << answer2.front() << std::endl;
    //std::cout << answer3.front() << std::endl;
    
    std::vector<cv::Point3d>::iterator p;
    std::vector<cv::Point3d>::iterator p_end;
    std::vector<uchar>::iterator d;
    std::vector<uchar>::iterator d_end;

    info_3D.get(3, p, p_end, d, d_end);
    
    
    for(;p != p_end; p++, d++)
    {
        std::cout << *p << std::endl;
        std::cout << *d << std::endl;
    }

    pcl::visualization::CloudViewer viewer("Cloudviewer");
    info_3D.show_cloud(viewer, 30);
}
