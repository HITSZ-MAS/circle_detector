#ifndef RANSACITER_H
#define RANSACITER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Geometry>

#include <random>
#include <ctime>


// class RansacIterBase
// {
// public:
//     RansacIterBase();
//     ~RansacIterBase();

// public:
//     int iter_time_;
//     double least_probability_;
//     double least_error_;

// public:
//     virtual void CalcResidual() =  0;
//     virtual void FitModel() = 0;

// };




class RansacFitCircle
{
public:
    RansacFitCircle();
    ~RansacFitCircle();

public:
    int iter_time_;
    double least_probability_;
    double least_error_;
    std::vector<double> model_;

    //random
    // std::random_device rand_device_;
    // std::uniform_int_distribution<unsigned> uniform_rand_device_;

    //for temp
    std::vector<double> FitCircle(std::vector<std::vector<double>> pts);
    double CalcResidual(const std::vector<double> circle,const std::vector<double> point);

    bool SolveRANSACFitCircle(const std::vector<std::vector<double>> pts , std::vector<double> &circle);
    void SetParams(int iter_time , double least_probability , double least_error);

public:
    int pre_inliers_;
    int pre_outliers_;
    
};









#endif