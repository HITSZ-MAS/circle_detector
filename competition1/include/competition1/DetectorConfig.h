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
        bool a = nh.getParam("/depthtest_node/color_fx",color_fx_);
        nh.getParam("/depthtest_node/color_fy",color_fy_);
        nh.getParam("/depthtest_node/color_cx",color_cx_);
        nh.getParam("/depthtest_node/color_cy",color_cy_);

        nh.getParam("/depthtest_node/depth_fx",depth_fx_);
        nh.getParam("/depthtest_node/depth_fy",depth_fy_);
        nh.getParam("/depthtest_node/depth_cx",depth_cx_);
        nh.getParam("/depthtest_node/depth_cy",depth_cy_);

        nh.getParam("/depthtest_node/real_circle_radius_big",real_circle_radius_big_);
        nh.getParam("/depthtest_node/real_circle_radius_small",real_circle_radius_small_);

        nh.getParam("/depthtest_node/camera_type",camera_type_);
        nh.getParam("/depthtest_node/detect_method",detect_method_);

        nh.getParam("/depthtest_node/resize_depth",resize_depth_);

        std::cout<<color_fx_<<std::endl;
        std::cout<<a<<std::endl;
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
    //param for circle detector
    double color_fx_;
    double color_fy_;
    double color_cx_;
    double color_cy_;

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


};



#endif