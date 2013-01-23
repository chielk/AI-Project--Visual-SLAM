#include "cloud.hpp"


template <class point>
Cloud<point>::Cloud()
{
   points = std::vector<point>();
   descriptors = std::vector<point>();
}

template <class point>
Cloud<point>::Cloud(std::vector<point> ps, std::vector<uchar> ds)
{
   points = ps;
   descriptors = ds;
}

template <class point>
void Cloud<point>::remove(int index, point &p, uchar &d)
{
   p = points.erase(points.begin()+index);
   d = descriptors.erase(descriptors.begin()+index);
}

template <class point>
void Cloud<point>::remove(int index)
{
   points.erase(points.begin()+index);
   descriptors.erase(descriptors.begin()+index);
}

template <class point>
void Cloud<point>::add(point p, uchar d)
{
   points.push_back(p);
   descriptors.push_back(d);
}

template <class point>
void Cloud<point>::remove_last(int n)
{
   if (n <= 0) {
      return;
   }
   points.erase(points.begin()+n);
   descriptors.erase(descriptors.begin()+n);
}
