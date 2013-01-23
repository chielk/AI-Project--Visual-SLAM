#ifndef CLOUD_H
#define CLOUD_H

#include <stdio.h>
#include <vector>

template <class point> class Cloud
{
   public:
      void remove(int index);
      void add(point, uchar);
      uchar get(point);
      point get(uchar);
      std::vector<point> get_points();
      std::vector<uchar> get_descriptors();
      Cloud();
      Cloud(std::vector<point>, std::vector<uchar>);
      ~Cloud();
   private:
      std::vector<point> points;
      std::vector<uchar> descriptors;
}

#endif
