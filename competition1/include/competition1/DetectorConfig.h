#ifndef COMP1_DETECTORCONFIG_H
#define COMP1_DETECTORCONFIG_H

#include <string>
#include <ros/ros.h>
#include <iostream>

class DetectorConfig
{
    // Config(const ros::NodeHandle &nh_priv)
    // {
    //     nh_priv.getParam("MapTopic", mapTopic);
    //     nh_priv.getParam("TargetTopic", targetTopic);
    // }

public:

    DetectorConfig(ros::NodeHandle &nh)
    {
        node_name_ = "/onlydetector_node";

        bool a = nh.getParam(node_name_+"/color_fx",color_fx_);
        nh.getParam(node_name_+"/color_fy",color_fy_);
        nh.getParam(node_name_+"/color_cx",color_cx_);
        nh.getParam(node_name_+"/color_cy",color_cy_);

        nh.getParam(node_name_+"/depth_fx",depth_fx_);
        nh.getParam(node_name_+"/depth_fy",depth_fy_);
        nh.getParam(node_name_+"/depth_cx",depth_cx_);
        nh.getParam(node_name_+"/depth_cy",depth_cy_);

        nh.getParam(node_name_+"/real_circle_radius_big",real_circle_radius_big_);
        nh.getParam(node_name_+"/real_circle_radius_small",real_circle_radius_small_);

        nh.getParam(node_name_+"/camera_type",camera_type_);
        nh.getParam(node_name_+"/detect_method",detect_method_);

        nh.getParam(node_name_+"/color_width",color_width_);
        nh.getParam(node_name_+"/color_height",color_height_);
        nh.getParam(node_name_+"/depth_width",depth_width_);
        nh.getParam(node_name_+"/depth_height",depth_height_);

        nh.getParam(node_name_+"/resize_depth",resize_depth_);
        nh.getParam(node_name_+"/show_image",show_image_);

        //for RANSAC
        nh.getParam(node_name_+"/use_RANSAC",use_RANSAC_);
        nh.getParam(node_name_+"/iter_time",RANSAC_iter_time_);
        nh.getParam(node_name_+"/least_error",RANSAC_least_error_);
        nh.getParam(node_name_+"/least_circle_ratio",RANSAC_least_circle_ratio_);

        //for detecting circle
        nh.getParam(node_name_+"/min_contours_area",min_contours_area_);
        nh.getParam(node_name_+"/min_points_in_camera",min_points_in_camera_);

        nh.getParam(node_name_+"/pixel_move",pixel_move_);
        nh.getParam(node_name_+"/pixel_move_max",pixel_move_max_);


        //for ROS topic
        nh.getParam(node_name_+"/color_image_topic",color_image_topic_);
        nh.getParam(node_name_+"/depth_image_topic",depth_image_topic_);
        nh.getParam(node_name_+"/circle_pose_topic",circle_pose_topic_);
        nh.getParam(node_name_+"/odom_topic",odom_topic_);
        nh.getParam(node_name_+"/use_odom",use_odom_);
        

        ROS_INFO("config loaded !!!!!!!!!!!!!!");
    }

    DetectorConfig()
    {
        ;
    }

    ~DetectorConfig()
    {
        ;
    }

public:
    std::string node_name_;


public:
    //param for circle detector
    double color_fx_;
    double color_fy_;
    double color_cx_;
    double color_cy_;

    double color_width_;
    double color_height_;
    double depth_width_;
    double depth_height_;

    // enum CameraType
    // {
    //     MONO=0,
    //     STEREO=1,
    //     RGBD=2
    // };
    std::string camera_type_;

    double depth_fx_;
    double depth_fy_;
    double depth_cx_;
    double depth_cy_;

    double real_circle_radius_big_;
    double real_circle_radius_small_;

    // enum DetectMethod
    // {
    //     DEPTH=0,
    //     COLOR=1
    // };
    std::string detect_method_;
    
    bool resize_depth_;  //default enlarge 2 times
    bool show_image_;
    bool use_odom_;


    //RANSAC
    bool use_RANSAC_;
    int RANSAC_iter_time_;
    double RANSAC_least_error_;
    double RANSAC_least_circle_ratio_;

    //detecting circle
    double min_contours_area_;
    int min_points_in_camera_;

    int pixel_move_;
    int pixel_move_max_;

    //for ROS topic
    std::string color_image_topic_;
    std::string depth_image_topic_;
    std::string circle_pose_topic_;
    std::string odom_topic_;

};



#endif