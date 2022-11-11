#include "pointcloud.h"


#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

PointCloud::PointCloud()
{
    this->height = 500;
    this->width = 500;
    
    this->cloud = cv::Mat::zeros(this->height, this->width, CV_64F);
    
}

void PointCloud::putToF(ToF_t tof)
{
    std::vector<double> distance, angle;
    
    angle.assign(tof.angle.begin(), tof.angle.end());
    distance.assign(tof.distance.begin(), tof.distance.end());

    int length = tof.length;
    cv::Mat cloud_temp = cv::Mat::zeros(this->height, this->width, CV_64F);
    //std::cout << std::dec << (signed)length << std::endl;
    if(length !=0)
    {
        for(int i=0; i<length; i++)
        {
            double dist = distance[i] * 0.025;
            double ang = angle[i];
            
            if(dist != 0)
            {
                                
                int x = (int)(dist * std::cos(ang*3.141592/180)) + this->width/2;
                int y = (int)(dist * std::sin(ang*3.141592/180)) + this->height/2;
                if(x < this->width && x >= 0 && y < this->height && y >= 0)
                {
                    //std::cout << std::dec << (signed)x << " " << (signed)y << std::endl;
                    /*axes.at<cv::Vec3b>(y,x)[0] = 0;
                    axes.at<cv::Vec3b>(y,x)[1] = 0;
                    axes.at<cv::Vec3b>(y,x)[2] = 0;*/
                    //canvas.at<cv::Vec3b>(y,x)[0] = 255;
                    //canvas.at<cv::Vec3b>(y,x)[1] = 255;
                    //canvas.at<cv::Vec3b>(y,x)[2] = 255;
                    cloud_temp.at<double>(y,x) = 1;
                }
            }
        }
        for(int y=0; y<this->height; y++)
        {
            for(int x=0; x<this->width; x++)
            {
                double p = this->cloud.at<double>(y,x)*(1-this->k) + cloud_temp.at<double>(y,x)*k;
                if(p > 1)
                    p = 1;
                if(p < 0)
                    p = 0;
                this->cloud.at<double>(y,x) = p;
            }
        }
    }
}

void PointCloud::putPoint(int _y, int _x)
{
    cv::Mat cloud_temp = cv::Mat::zeros(this->height, this->width, CV_64F);
    
    int y = _y + this->height/2;
    int x = _x + this->width/2;

    if(x < this->width && x >= 0 && y < this->height && y >= 0)
    {
        cloud_temp.at<double>(y,x) = 1;
    }
    for(int y=0; y<this->height; y++)
    {
        for(int x=0; x<this->width; x++)
        {
            double p = this->cloud.at<double>(y,x)*(1-this->k) + cloud_temp.at<double>(y,x)*k;
            if(p > 1)
                p = 1;
            if(p < 0)
                p = 0;
            this->cloud.at<double>(y,x) = p;
        }
    }
}

void PointCloud::plot()
{
    
    while(true)
    {
        
        
        
        cv::Mat axes = this->getMap();
        
        cv::imshow("Plot", axes);
        
        if(cv::waitKey(1) == 27)
            break;
        
    }
    cv::destroyAllWindows();
    
    std::cout << "Plot exit" << std::endl;
}
cv::Mat PointCloud::getMap()
{
    cv::Mat map = cv::Mat(this->height, this->width, CV_8UC3, cv::Scalar(255, 255, 255));
    for(int y=0; y<this->height; y++)
    {
        for(int x=0; x<this->width; x++)
        {            
            map.at<cv::Vec3b>(y,x)[0] = (uchar)((1-this->cloud.at<double>(y,x))*255);
            map.at<cv::Vec3b>(y,x)[1] = (uchar)((1-this->cloud.at<double>(y,x))*255);
            map.at<cv::Vec3b>(y,x)[2] = (uchar)((1-this->cloud.at<double>(y,x))*255);
        }
    }
    
    return map;
}