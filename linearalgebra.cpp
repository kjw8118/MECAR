#include "linearalgebra.h"

#include <cmath>

#include <opencv2/opencv.hpp>

cv::Vec3d LinearAlgebra::RotateX(cv::Vec3d vec, double rad)
{
    double c = std::cos(rad);
    double s = std::sin(rad);
    double buf[9] = {
        1, 0, 0,
        0, c, -s,
        0, s, c
    };
    cv::Mat T(3, 3, CV_64F, buf);
    cv::Mat vec2 = T*cv::Mat(vec);
    return cv::Vec3d((double*)vec2.data);
}
cv::Vec3d LinearAlgebra::RotateY(cv::Vec3d vec, double rad)
{
    double c = std::cos(rad);
    double s = std::sin(rad);
    double buf[9] = {
        c, 0, s,
        0, 1, 0,
        -s, 0, c
    };
    cv::Mat T(3, 3, CV_64F, buf);
    cv::Mat vec2 = T*cv::Mat(vec);
    return cv::Vec3d((double*)vec2.data);
}
cv::Vec3d LinearAlgebra::RotateZ(cv::Vec3d vec, double rad)
{
    double c = std::cos(rad);
    double s = std::sin(rad);
    double buf[9] = {
        c, -s, 0,
        s, c, 0,
        0, 0, 1
    };
    cv::Mat T(3, 3, CV_64F, buf);
    cv::Mat vec2 = T*cv::Mat(vec);
    return cv::Vec3d((double*)vec2.data);
}
