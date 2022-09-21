#ifndef __FRONT_CAMERA_H__
#define __FRONT_CAMERA_H__

#include <opencv2/opencv.hpp>
using namespace cv;

class FrontCamera
{
private:
    Mat img;
    VideoCapture cap;

public:
    FrontCamera(/* args */);
    int view();
    void run();
    ~FrontCamera();
};

#endif