#ifndef __LINEARALGEBRA_H__
#define __LINEARALGEBRA_H__


#include <opencv2/opencv.hpp>

namespace LinearAlgebra
{
    cv::Vec3d RotateX(cv::Vec3d vec, double rad);
    cv::Vec3d RotateY(cv::Vec3d vec, double rad);
    cv::Vec3d RotateZ(cv::Vec3d vec, double rad);
};




#endif