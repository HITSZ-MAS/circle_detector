#include <iostream>
#include "competition1/ColorDetect.h"
#include "competition1/DetectorConfig.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>

using namespace std;

//#define HOUGHCIRCLE
//#define CONTUORS
//#define ERZHIHUA
//#define FITCIRCLE
#define COLOR


class OnlyDetector
{
  public:
    OnlyDetector(ros::NodeHandle &nh)
    {
        nh_=nh;
        pCDetector = std::make_shared<ColorDetector>(nh_);
        color_sub=nh_.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw",1,&OnlyDetector::ImageCallback,this);
        //color_sub=nh_.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw",1,&OnlyDetector::ImageCallback,this);
        pconfig_ = new DetectorConfig(nh_);

    }
  public:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/airsim_node/drone_1/circle_pose",1);
    //ros::Subscriber depth_sub = nh_.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_center/DepthPlanar",1,&OnlyDetector::DepthCallback,this);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> syncpolicy;//时间戳对齐规则
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, nav_msgs::Odometry> syncpolicy;//时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    ros::Subscriber color_sub;

    DetectorConfig *pconfig_;

    void ImageCallback(const sensor_msgs::Image::ConstPtr &msgs);
    //void DepthCallback(sensor_msgs::Image msgs);
    std::shared_ptr<ColorDetector> pCDetector;
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"depthtest");
  ros::NodeHandle nh;
  OnlyDetector detector(nh);
  ros::spin();

  return 0;
}

void OnlyDetector::ImageCallback(const sensor_msgs::Image::ConstPtr &msgs)
{
    //回调函数实现
    //图片转换
    cv::Mat output,circle;
    cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
    try
    {
        //若成功捕获到消息，那么对ROS类型的图片消息进行转换，转换为OpenCV的格式
        //cv_ptr = cv_bridge::toCvCopy(*msgs, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_8UC3);
        //cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        //若本次捕获消息失败，则catch error ，输出错误信息并且返回，进行下一次消息捕获。
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // cv::Mat floatMat(pconfig_->depth_width_,pconfig_->depth_height_,CV_32FC1);
    // cv::Mat ucharMatScaled;
    // floatMat = cv_ptr->image.clone();
    // floatMat.clone().convertTo(floatMat,CV_32FC1);
    // floatMat = floatMat/1000.0;
    // floatMat.clone().convertTo(ucharMatScaled, CV_8UC1, 255, 0); 
    // cv::resize(ucharMatScaled,ucharMatScaled,cv::Size(640,480));
    //imshow("1",ucharMatScaled);
    // std::cout<<ucharMatScaled<<std::endl;

#ifdef HOUGHCIRCLE
    //std::vector<cv::Vec3f> circles;
    //bool CircleExist = pCDetector->HoughFindCircle(cv_ptr->image,circles);
    //pCDetector->CirclePose(circles , ColorDetector::DEPTH_TYPE);
    cv::Mat gray,color,erzhi;
    color = cv_ptr->image.clone();
    cv::cvtColor(color,gray,cv::COLOR_BGR2GRAY);

    
    cv::adaptiveThreshold(gray,erzhi,255,cv::THRESH_BINARY, cv::ADAPTIVE_THRESH_GAUSSIAN_C,3,5);
    cv::imshow("1",erzhi);
#endif

#ifdef CONTUORS
    std::vector<cv::Rect> boundRects;
    bool RectExsit = pCDetector->FindContoursDepth(ucharMatScaled , boundRects,floatMat);
    if(RectExsit)
    {
        pCDetector->CirclePose(boundRects); 
    }

#endif

#ifdef FITCIRCLE
    std::vector<cv::Rect> boundRects;
    bool RectExsit = pCDetector->FindContoursDepth(ucharMatScaled , boundRects,floatMat);
    // std::cout<<"fit Circle"<<std::endl;
    //std::cout<<floatMat<<std::endl;
    cv::imshow("1",ucharMatScaled);
    cv::waitKey(30);
#endif

#ifdef COLOR
    cv::Mat temp_RGB;
    cv::cvtColor(cv_ptr->image,temp_RGB,cv::COLOR_BGRA2RGB);
    cv::imshow("c",temp_RGB);
    pCDetector->Detect(temp_RGB,ColorDetector::BLUE,output);
    // cv::Rect boundRect;
    // bool RectExist = pCDetector->DrawRect(output, boundRect);
    // std::vector<cv::Vec3f> circles;
    // bool CircleExist = pCDetector->DrawCircle(output,circles);

    // if(/*RectExist == true*/CircleExist)
    // {
    // //pCDetector->CirclePose(boundRect);
    // pCDetector->CirclePose(circles , 1);
    // }
    // else
    // {
    // //if we cannot detect circle , we do this

    // }

#endif

    cv::waitKey(10);
}



