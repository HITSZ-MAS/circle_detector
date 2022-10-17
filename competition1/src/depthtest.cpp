#include <iostream>
#include "competition1/ColorDetect.h"
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
#define FITCIRCLE


class CircleDetector
{
  public:
    CircleDetector(ros::NodeHandle &nh)
    {
        nh_=nh;
        pCDetector = std::make_shared<ColorDetector>(nh_);
        color_sub.subscribe(nh_, "/airsim_node/drone_1/front_center/DepthPlanar", 1);
        odom_sub.subscribe(nh_, "/airsim_node/drone_1/odom_local_enu", 1);
        sync_.reset(new Sync(syncpolicy(10), color_sub, odom_sub));
        sync_->registerCallback(boost::bind(&CircleDetector::ImageOdomCallback, this, _1, _2));

    }
  public:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub  = nh_.advertise<std_msgs::Float64MultiArray>("/airsim_node/drone_1/circle_pose",1);
    //ros::Subscriber depth_sub = nh_.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_center/DepthPlanar",1,&CircleDetector::DepthCallback,this);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> syncpolicy;//时间戳对齐规则
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, nav_msgs::Odometry> syncpolicy;//时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    message_filters::Subscriber<sensor_msgs::Image> color_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

    void ImageOdomCallback(const sensor_msgs::Image::ConstPtr &msgs, const nav_msgs::Odometry::ConstPtr &odom);
    //void DepthCallback(sensor_msgs::Image msgs);
    std::shared_ptr<ColorDetector> pCDetector;
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"colortest");
  ros::NodeHandle nh;
  CircleDetector detector(nh);
  ros::spin();

  return 0;
}

void CircleDetector::ImageOdomCallback(const sensor_msgs::Image::ConstPtr &msgs, const nav_msgs::Odometry::ConstPtr &odom)
{
    //回调函数实现
    //图片转换
    cv::Mat output,circle;
    cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
    try
    {
        //若成功捕获到消息，那么对ROS类型的图片消息进行转换，转换为OpenCV的格式
        //cv_ptr = cv_bridge::toCvCopy(*msgs, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        //若本次捕获消息失败，则catch error ，输出错误信息并且返回，进行下一次消息捕获。
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat floatMat(320,240,CV_32FC1);
    cv::Mat ucharMatScaled;
    floatMat = cv_ptr->image.clone();
    //floatMat = floatMat/10.0;
    floatMat.clone().convertTo(ucharMatScaled, CV_8UC1, 255, 0); 
    cv::resize(ucharMatScaled,ucharMatScaled,cv::Size(640,480));
    //imshow("1",ucharMatScaled);
    // std::cout<<ucharMatScaled<<std::endl;

#ifdef HOUGHCIRCLE
    std::vector<cv::Vec3f> circles;
    bool CircleExist = pCDetector->HoughFindCircle(ucharMatScaled,circles);
    pCDetector->CirclePose(circles , ColorDetector::DEPTH_TYPE);
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
#endif

    //pub pose
    std::vector<std::vector<double>> Circles_Pose;
    pCDetector->GetCirclePose(Circles_Pose);

    int circlenum = Circles_Pose.size();
    if(circlenum<=0)
    {
        cv::waitKey(10);
        return;
    }

    for(int i=0;i<circlenum;++i)
    {
        std::vector<double> pose = Circles_Pose[i];
        std_msgs::Float64MultiArray msg;
        
        if(RectExsit==false)
        {
            cv::waitKey(50);
            return;
        }

        Eigen::Vector3d point(-pose[1], pose[0], pose[2]);
        Eigen::Vector3d odom_p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        Eigen::Quaterniond odom_q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        point = odom_q.toRotationMatrix() * point + odom_p;

        msg.data.push_back(point(0));
        msg.data.push_back(point(1));
        msg.data.push_back(point(2));
        msg.data.push_back(pose[3]);
        pose_pub.publish(msg);
        cv::waitKey(10);
    }

}

// void CircleDetector::DepthCallback(sensor_msgs::Image msgs)
// {
//   cv::Mat output,circle;
//   cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
//   std::cout<<"depth"<<std::endl;
//   try
//   {
//     //若成功捕获到消息，那么对ROS类型的图片消息进行转换，转换为OpenCV的格式
//     //cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::BGR8);
//     cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_32FC1);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     //若本次捕获消息失败，则catch error ，输出错误信息并且返回，进行下一次消息捕获。
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
//   //显示图像，cv_ptr->image为OpenCV格式的图像，可以直接显示
//   cv::Mat floatMat(320,240,CV_32FC1);
//   cv::Mat ucharMatScaled;
//   floatMat = cv_ptr->image.clone();
//   floatMat = floatMat/10.0;
//   floatMat.convertTo(ucharMatScaled, CV_8UC1, 255, 0); 
//   imshow("1",ucharMatScaled);
//  // std::cout<<ucharMatScaled<<std::endl;

//   std::vector<cv::Vec3f> circles;
//   pCDetector->HoughFindCircle(ucharMatScaled,circles);
//   cv::waitKey(50);
// }


