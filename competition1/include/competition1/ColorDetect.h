#ifndef COLORDETECT_H
#define COLORDETECT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include "competition1/DetectorConfig.h"
#include "competition1/RansacIter.h"

class ColorDetector
{
    public:
        ColorDetector();
        ColorDetector(ros::NodeHandle &nh);
        ~ColorDetector(){;};

        ros::NodeHandle nh_;

        //int COLOR[1][6] = {9,16,43,255,46,255};  //can detect first two OK
        //int COLOR[1][6] = {0,10,43,255,46,255};  //can detect the third OK
        //int COLOR[1][6] = {17,25,43,255,46,255}; // can detect road
        //int COLOR[1][6] = {17,19,43,255,46,255}; // can detect 5 5  with road
        int COLOR[1][6] = {0,16,43,255,46,255}; // no
        enum COLORTYPE
        {
            NONE=-1,
            RED=0
        };


        void Initialize();
        bool Detect(cv::Mat inputImg , int colortype , cv::Mat &outputImg);
        void HoughTranslate(cv::Mat& image);
        bool DrawRect(cv::Mat inputImg,cv::Rect &boundRect);
        bool HoughFindCircle(cv::Mat& input , std::vector<cv::Vec3f> &circles);
        bool DrawCircle(cv::Mat inputImg , std::vector<cv::Vec3f> &circles);
        void CirclePose(cv::Rect &rect);
        void CirclePose(std::vector<cv::Vec3f> circles , int DETECTTYPE);
        void CirclePose(std::vector<cv::Rect> &rects);
        bool isInSquare(double centerx,double centery);
        bool FindContoursDepth(cv::Mat inputImg,std::vector<cv::Rect> &boundRect,cv::Mat depth);
        bool IsOutOfRange(int width , int height , int row,int col); //y <height ; x < width
        
        std::vector<double> FitCircle(std::vector<std::vector<double>> pts);

        std::vector<double> Pixel2Camera(double x,double y,double depth);


    public:
    //for camera
        double fx,fy,cx,cy;
        double depth_fx_;
        double depth_fy_;
        double depth_cx_;
        double depth_cy_;
        cv::Mat K;
        double realRadius;

        double color_width_;
        double color_height_;
        double depth_width_;
        double depth_height_;

    public:
    //for color detection
        int circle_num_;

    public:
    //for circle pose
        double PoseX_;
        double PoseY_;
        double PoseZ_;
        double PoseYaw_;
        double Z_From_Depth_;
        void GetCirclePose(double &x,double &y , double &z , double &yaw);
        void GetCirclePose(std::vector<std::vector<double>> &p);
        enum detecttype
        {
            DEPTH_TYPE = 0,
            COLOR_TYPE = 1
        };

        std::vector<std::vector<double>> Circles_Pose_;
        std::vector<std::vector<double>> Circles_Points_Camera_;
        std::vector<std::vector<double>> Fitted_Circles_;

    public:
    //for params
        DetectorConfig *config_;
        bool is_resize_depth_;

        RansacFitCircle Rfc_;
        
};







#endif
