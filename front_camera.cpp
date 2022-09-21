#include "front_camera.h"

#include <opencv2/opencv.hpp>
using namespace cv;


FrontCamera::FrontCamera(/* args */)
    : cap(0, CAP_V4L)
{
    this->cap.set(CAP_PROP_FRAME_WIDTH, 640);
    this->cap.set(CAP_PROP_FRAME_HEIGHT, 480);
}

FrontCamera::~FrontCamera()
{
}

int FrontCamera::view()
{
    this->cap >> this->img;
    imshow("View", this->img);
    return waitKey(1);
        
}

void FrontCamera::run()
{
    int val = 0;
    while(val != 27)
    {
        val = this->view();        
    }
}