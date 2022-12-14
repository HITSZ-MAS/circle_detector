#include <iostream>
#include "competition1/ColorDetect.h"
#include "competition1/DetectorConfig.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

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
//#define DEPTH
#define COLOR


class OnlyDetector
{
  public:
    OnlyDetector(ros::NodeHandle &nh)
    {
        nh_=nh;
        pCDetector = std::make_shared<ColorDetector>(nh_);
        pconfig_ = new DetectorConfig(nh_);
        color_sub.subscribe(nh_,pconfig_->color_image_topic_,1);
        depth_sub.subscribe(nh_,pconfig_->depth_image_topic_,1);
        pose_pub  = nh_.advertise<std_msgs::Float64MultiArray>(pconfig_->circle_pose_topic_,1);
        point_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("/drone_1/point_cloud",100);
        center_pub = nh_.advertise<sensor_msgs::PointCloud>("/drone_1/circle_center",100);

        odom_sub=nh_.subscribe<nav_msgs::Odometry>(pconfig_->odom_topic_,1,&OnlyDetector::OdomCallback,this);

        sync_.reset(new Sync(syncpolicy(10), color_sub, depth_sub));
        sync_->registerCallback(boost::bind(&OnlyDetector::ColorDepthCallback, this, _1, _2));


    }
  public:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub; 
    ros::Publisher point_cloud_pub;
    ros::Publisher center_pub;
    ros::Subscriber odom_sub;
    //ros::Subscriber depth_sub = nh_.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_center/DepthPlanar",1,&OnlyDetector::DepthCallback,this);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncpolicy;//?????????????????????
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, nav_msgs::Odometry> syncpolicy;//?????????????????????
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    message_filters::Subscriber<sensor_msgs::Image> color_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    
    void ColorDepthCallback(const sensor_msgs::Image::ConstPtr &color, 
                                                            const sensor_msgs::Image::ConstPtr &depth);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom);

    DetectorConfig *pconfig_;

    std::shared_ptr<ColorDetector> pCDetector;

     //odom
    Eigen::Vector3d point;
    Eigen::Vector3d odom_p;
    Eigen::Matrix3d odom_R;    

    std::queue<std::pair<ros::Time,Eigen::Vector3d>> odom_p_queue;
    std::queue<std::pair<ros::Time,Eigen::Matrix3d>> odom_R_queue;
};


int main(int argc,char** argv)
{
  ros::init(argc,argv,"depthtest");
  ros::NodeHandle nh;
  OnlyDetector detector(nh);
  ros::spin();

  return 0;
}

void OnlyDetector::ColorDepthCallback(const sensor_msgs::Image::ConstPtr &color,
                                                 const sensor_msgs::Image::ConstPtr &depth)
{
    //??????????????????
    //????????????
    cv::Mat output,circle;

    cv_bridge::CvImagePtr color_cv_ptr,depth_cv_ptr;  //CvImagePtr for color img and depth img.
    try
    {
        //for color
        //????????????????????????????????????ROS?????????????????????????????????????????????OpenCV?????????
        color_cv_ptr = cv_bridge::toCvCopy(*color, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //?????????????????????????????????catch error ??????????????????????????????????????????????????????????????????
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        //for depth
        //????????????????????????????????????ROS?????????????????????????????????????????????OpenCV?????????
        depth_cv_ptr = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        //?????????????????????????????????catch error ??????????????????????????????????????????????????????????????????
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat color_image,depth_image_f;
    color_image=color_cv_ptr->image.clone();
    depth_image_f = depth_cv_ptr->image.clone();
    depth_image_f = depth_image_f / 1000.0;
    //depth_image_f.clone().convertTo(depth_image_f, CV_8UC1, 255, 0); 


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

#ifdef COLOR
    //cv::resize(color_image,color_image,cv::Size(848,480));
    pCDetector->Detect(color_image,ColorDetector::BLUE,output);
    std::vector<cv::Rect> boundRects;
    // std::vector<cv::Vec3f> circles;
    // pCDetector->HoughFindCircle(output,circles);
    bool RectExsit = pCDetector->FindContoursDepth(output , boundRects,depth_image_f);
    cv::imshow("depth",depth_image_f);
    cv::imshow("color",color_image);
#endif

     //pub pose
    std::vector<std::vector<double>> Circles_Pose;
    pCDetector->GetCirclePose(Circles_Pose);

    int circlenum = Circles_Pose.size();
    if(circlenum<=0)
    {
        cv::waitKey(20);
        return;
    }

    for(int i=0;i<circlenum;++i)
    {
        std::vector<double> pose = Circles_Pose[i];
        std_msgs::Float64MultiArray msg;
        
        if(RectExsit==false)
        {
            cv::waitKey(20);
            return;
        }

        if(pconfig_->use_odom_)
        {
            Eigen::Vector3d point(pose[0], pose[1], pose[2]);
            ros::Time t_now=depth->header.stamp;
            while(1)
            {
                if(odom_R_queue.empty())
                {
                    ROS_WARN("no Odometry!!! Please check!!");
                    return;
                }
                if(abs((t_now-odom_R_queue.front().first).toSec())<0.1)
                {
                    odom_R=odom_R_queue.front().second;
                    odom_p=odom_p_queue.front().second;
                    odom_R_queue.pop();
                    odom_p_queue.pop();
                    break;
                }
                odom_R_queue.pop();
                odom_p_queue.pop();
            }

            // Eigen::Vector3d point(-pose[1], pose[0], pose[2]);
            // Eigen::Vector3d odom_p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
            // Eigen::Quaterniond odom_q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
            // point = odom_q.toRotationMatrix() * point + odom_p;

            if(odom_R.isZero())
            {
                cv::waitKey(10);
                return;
            }

            point  = odom_R*point+odom_p;

            msg.data.push_back(pose[0]);
            msg.data.push_back(pose[1]);
            msg.data.push_back(pose[2]);
            msg.data.push_back(pose[3]);
            pose_pub.publish(msg);

        }

 
        //pub point cloud
        sensor_msgs::PointCloud margin_cloud;
        sensor_msgs::PointCloud center_point;
        margin_cloud.header = color->header;

        int points_num=pCDetector->Circles_Points_Camera_.size();
        std::vector<std::vector<double>> circle_points = pCDetector->Circles_Points_Camera_;
        for(int j=0;j<points_num;++j)
        {
            geometry_msgs::Point32 p32;
            p32.x=circle_points[j][0];
            p32.y=circle_points[j][1];
            p32.z=circle_points[j][2];
            margin_cloud.points.push_back(p32);
        }
        geometry_msgs::Point32 p_center;
        p_center.x=pose[0];
        p_center.y=pose[1];
        p_center.z=pose[2];

        margin_cloud.points.push_back(p_center);

        point_cloud_pub.publish(margin_cloud);
        ROS_ERROR("pose:%f , %f , %f",pose[0],pose[1],pose[2]);


    }


    cv::waitKey(10);
}

void OnlyDetector::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    if(!pconfig_->use_odom_)
    {
        return;
    }
    double odom_p_x=odom->pose.pose.position.x;
    double odom_p_y=odom->pose.pose.position.y;
    double odom_p_z=odom->pose.pose.position.z;

    Eigen::Vector3d odom_p_temp(odom_p_x,odom_p_y,odom_p_z);

    Eigen::Quaterniond odom_q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                                    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    
    Eigen::Matrix3d odom_R_temp=odom_q.toRotationMatrix();
    odom_p_queue.push(std::make_pair(odom->header.stamp,odom_p_temp));
    odom_R_queue.push(std::make_pair(odom->header.stamp,odom_R_temp));

    // odom_q.w=odom->pose.pose.orientation.w;
    // odom_q(1)=odom->pose.pose.orientation.w;
    // odom_q(2)=odom->pose.pose.orientation.w;
    // odom_q(3)=odom->pose.pose.orientation.w;
}

// void OnlyDetector::DepthCallback(sensor_msgs::Image msgs)
// {
//   cv::Mat output,circle;
//   cv_bridge::CvImagePtr cv_ptr;  //????????????CvImagePtr
//   std::cout<<"depth"<<std::endl;
//   try
//   {
//     //????????????????????????????????????ROS?????????????????????????????????????????????OpenCV?????????
//     //cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::BGR8);
//     cv_ptr = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::TYPE_32FC1);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     //?????????????????????????????????catch error ??????????????????????????????????????????????????????????????????
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
//   //???????????????cv_ptr->image???OpenCV????????????????????????????????????
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


