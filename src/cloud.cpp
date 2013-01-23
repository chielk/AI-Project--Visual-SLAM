#include "cloud.hpp"

Cloud::Cloud()
{
   points = std::vector<point>();
   descriptors = std::vector<point>();
}

Cloud::Cloud(std::vector<point> ps, std::vector<uchar> ds)
{
   points = ps;
   descriptors = ds;
}

void Cloud::remove(int index, point &p, uchar &d)
{
   p = points.erase(points.begin()+index);
   d = descriptors.erase(descriptors.begin()+index);
}

void Cloud::remove(int index)
{
   points.erase(points.begin()+index);
   descriptors.erase(descriptors.begin()+index);
}

void Cloud::add(point p, uchar d)
{
   points.push_back(p);
   descriptors.push_back(d);
}

void Cloud::remove_last(int n)
{
   points.erase(points.begin()+index);
   descriptors.erase(descriptors.begin()+index);
}
