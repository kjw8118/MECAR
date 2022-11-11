#ifndef __POINTCLOUD_H__
#define __POINTCLOUD_H__

#include <opencv2/opencv.hpp>
#include <vector>

typedef struct _ToF_t
{        
    std::vector<double> angle;
    std::vector<double> distance;
    int length;
} ToF_t;

class PointCloud
{
private:
    cv::Mat cloud;
    int height;
    int width;
    double k = 0.2;
public:
    PointCloud();
    //~PointCloud();
    void putToF(ToF_t tof);
    void putPoint(int _y, int _x);
    void plot();
    cv::Mat getMap();
    //int getMap();
};








#endif