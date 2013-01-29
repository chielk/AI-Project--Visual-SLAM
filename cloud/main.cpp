#include "cloud.cpp"
#include <iostream>

int main ()
{
    Cloud<cv::Point3f> info_3D;

    std::vector<cv::Point3f> pts;
    std::vector<uchar> dscs;
    std::vector<int> fs;
    cv::Point3f pt(1, 2, 3);
    uchar dsc = 123;
    int frame = 3;

    pts.push_back(pt);
    dscs.push_back(dsc);
    fs.push_back(frame);

    // Push 3 times a vector of points to the structure
    info_3D.add(pts, dscs, fs);
    info_3D.add(pts, dscs, fs);
    info_3D.add(pts, dscs, fs);

    // Create answer structures
    std::vector<int> answer1;
    std::vector<uchar> answer2;
    std::vector<cv::Point3f> answer3;
    info_3D.get_frames(answer1);
    info_3D.get_descriptors(answer2);
    info_3D.get_points(answer3);
    // Print last structure
    std::cout << answer1.front() << std::endl;
    std::cout << answer2.front() << std::endl;
    std::cout << answer3.front() << std::endl;
    
    std::vector<cv::Point3f>::iterator p_begin;
    std::vector<cv::Point3f>::iterator p_end;
    std::vector<uchar>::iterator d_begin;
    std::vector<uchar>::iterator d_end;

    info_3D.get(3, p_begin, p_end, d_begin, d_end);
    
    
/*
    for(;p_begin != p_end; p_begin++; d_begin_end;)
    {
        std::cout << *p_begin << std::endl;
        std::cout << *d_begin << std::endl;
    }

*/
}
