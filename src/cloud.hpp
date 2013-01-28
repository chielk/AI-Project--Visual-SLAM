#ifndef CLOUD_H
#define CLOUD_H
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <vector>

template <class point> class Cloud
{
   public:
      void remove(int index);
      void remove(int, point &p, uchar &d, int &f);
      void remove_last(int);
      void remove_frame(int);
      void add(point, uchar, int);
      void add(std::vector<point>, std::vector<uchar>, std::vector<int>);
      void get(int frame,
              typename std::vector<point>::iterator &p_begin,
              typename std::vector<point>::iterator &p_end,
              std::vector<uchar>::iterator &d_begin,
              std::vector<uchar>::iterator &d_end);
      void get_points(const std::vector<point> &pts);
      void get_descriptors(const std::vector<uchar> &dscs);
      void get_frames(const std::vector<int> &fs);
      Cloud();
      Cloud(std::vector<point>, std::vector<uchar>);
      ~Cloud();
   private:
      std::vector<point> points;
      std::vector<uchar> descriptors;
      std::vector<int> frames;
};

#endif
