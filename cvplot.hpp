#ifndef __CVPLOT_H__
#define __CVPLOT_H__

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>


class CVPlot
{
private:
    //cv::Mat fig;
    int height = 500;
    int width = 500;
    
public:
    template<typename T>
    void plot(std::vector<T>& _x, std::vector<T>& _y);
    
    template<typename T>        
    void plot(T _x, T _y);


};

template<typename T>
void CVPlot::plot(std::vector<T>& _x, std::vector<T>& _y)
{    
    int data_length = std::min(_x.size(), _y.size());
    std::vector<T> x, y;
    for(int i=0; i<data_length; i++)
    {
        x.push_back(_x[i]);
        y.push_back(_y[i]);
    }
    while(data_length != x.size())
    {
        x.pop_back();
    }
    while(data_length != y.size())
    {
        y.pop_back();
    }
    
    T xmin = *std::min_element(x.begin(), x.end());
    T xmax = *std::max_element(x.begin(), x.end());    
    double xscalar = (double)this->width/(double)(xmax-xmin);

    T ymin = *std::min_element(y.begin(), y.end());
    T ymax = *std::max_element(y.begin(), y.end());    
    double yscalar = (double)this->height/(double)(ymax-ymin);

    int axis_scale = 20;
    std::vector<T> axis_x,  axis_y;
    T xunit = (xmax - xmin)/axis_scale;
    T yunit = (ymax - ymin)/axis_scale;
    for(int i=0; i<axis_scale; i++)
    {
        axis_x.push_back((i * xunit)*xscalar);
        axis_y.push_back((i * yunit)*yscalar);
    }
    

    cv::Mat axes(this->height, this->width, CV_8UC3, cv::Scalar(255, 255, 255));
    for(int i=0; i<data_length; i++)
    {        
        cv::circle(axes, cv::Point((int)((y[i] - ymin)*yscalar), (int)((x[i] - xmin)*xscalar)), 1, cv::Scalar(0,0,0), -1);        
    }
    std::cout << std::endl;

    cv::flip(axes, axes, 1);
    cv::Mat figure_upper, figure_lower, figure;
    int bezel = 100;
    cv::Scalar canvas_color(220, 220, 220);
    cv::Mat canvas_axis_x(bezel, this->width, CV_8UC3, canvas_color);
    cv::Mat canvas_axis_y(this->height, bezel, CV_8UC3, canvas_color);
    cv::Mat canvas_corner(bezel, bezel, CV_8UC3, canvas_color);
    cv::Mat canvas_upper(bezel, this->width + bezel, CV_8UC3, canvas_color);
    cv::Mat canvas_right(this->height + bezel*2, bezel, CV_8UC3, canvas_color);
    for(int i=0; i<axis_scale; i++)
    {
        
        cv::line(canvas_axis_x, cv::Point(axis_x[i], 0), cv::Point(axis_x[i], 10), cv::Scalar(0,0,0));
        cv::line(canvas_axis_y, cv::Point(0, axis_y[i]), cv::Point(10, axis_y[i]), cv::Scalar(0,0,0));
        
    }
    
    cv::flip(canvas_axis_y, canvas_axis_y, -1);
    
    cv::hconcat(canvas_axis_y, axes, figure_upper);
    cv::vconcat(canvas_upper, figure_upper, figure_upper);
    cv::hconcat(canvas_corner, canvas_axis_x, figure_lower);    
    cv::vconcat(figure_upper, figure_lower, figure);
    cv::hconcat(figure, canvas_right, figure);
    std::cout << "Test " << std::endl;

    cv::imshow("Figure", figure);
    cv::waitKey(0);

}

template<typename T>
void CVPlot::plot(T x, T y)
{
    std::vector<T> vx = {x};
    std::vector<T> vy = {y};
    this->plot(vx, vy);
}


#endif