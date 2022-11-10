#include "competition1/RansacIter.h"

RansacFitCircle::RansacFitCircle()
{
    iter_time_ = 200;
    least_error_ = 0.07; //meter
    least_probability_ = 0.99;
    least_circle_ratio_ = 0.90;
}

RansacFitCircle::~RansacFitCircle()
{
    ;
}

void RansacFitCircle::SetParams(int iter_time , double least_probability , double least_error,double least_circle)
{
    iter_time_=iter_time;
    least_error_=least_error;
    least_probability_=least_probability;
    least_circle_ratio_=least_circle;
}


bool RansacFitCircle::SolveRANSACFitCircle(const std::vector<std::vector<double>> pts , std::vector<double> &circle)
{
    pre_inliers_ = 0;

    int each_num=5;
    int pts_num=pts.size();
    std::default_random_engine e(time(NULL));
    std::uniform_int_distribution<int> uniform_rand_device_(0,pts_num-1);

    if(pts_num<50)
    {
        ROS_WARN("not enough points for fitting circle");
        return false;
    }
    for(int i=0;i<iter_time_;++i)
    {
        int each_outliers=0;
        int each_inliers;
        std::vector<int> rand_num;
        std::vector<std::vector<double>> each_pts;
        std::vector<double> each_circle;
        for(int j=0;j<each_num;j++)
        {
            int r=uniform_rand_device_(e);
            rand_num.push_back(r);
            each_pts.push_back(pts[r]);

            each_circle=FitCircle(each_pts);

        }
        
        //calc error for each points
        for(int j=0 ; j<pts_num;++j)
        {
            double err=CalcResidual(each_circle,pts[j]);
            if(err>least_error_)
            {
                //outliers
                each_outliers++;
            }
        }
        each_inliers = pts_num - each_outliers;

        if(each_inliers>pre_inliers_)
        {
            pre_inliers_ = each_inliers;
            model_=each_circle;
        }
    }
    double ratio = double(pre_inliers_)/double(pts_num);
    if(ratio<least_circle_ratio_)
    {
        ROS_WARN("this circle is not good!!!");
        circle = model_;
        return false;
    }
    else
    {
        std::cout<<ratio<<std::endl;
    }
    circle = model_;
    return true;
}

double RansacFitCircle::CalcResidual(const std::vector<double> circle,const std::vector<double> point)
{
    //model c(3) a(3) r(1) all 7
    Eigen::Vector3d center(circle[0],circle[1],circle[2]);
    Eigen::Vector3d plant(circle[3],circle[4],circle[5]);
    double r=circle[6];

    double point_to_plant;
    double point_to_center;
    Eigen::Vector3d pt(point[0],point[1],point[2]);
    point_to_plant = std::abs(plant(0)*point[0]+plant(1)*point[1]+plant(2)*point[2]-1)/std::sqrt(plant.dot(plant));
    point_to_center = std::abs((pt-center).norm()-r);

    return point_to_center + point_to_center;
}




std::vector<double> RansacFitCircle::FitCircle(std::vector<std::vector<double>> pts)
{
    std::vector<double> circle;

    int num = pts.size();

    int dim = 3;

    Eigen::MatrixXd M(num, dim);

    for (int i = 0; i < num; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            M(i, j) = pts[i][j];
        }
    }

    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);

    //式（6）
    Eigen::MatrixXd A = (M.transpose()*M).inverse()*M.transpose()*L1;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);

    for (int i = 0; i < num - 1; i++)
    {
        B.row(i) = M.row(i + 1) - M.row(i);
    }

    Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
    for (int i = 0; i < num - 1; i++)
    {
        L2(i) = (M(i + 1, 0)*M(i + 1, 0) + M(i + 1, 1)*M(i + 1, 1) + M(i + 1, 2)*M(i + 1, 2)
            - (M(i, 0)*M(i, 0) + M(i, 1)*M(i, 1) + M(i, 2)*M(i, 2))) / 2.0;
    }

    Eigen::MatrixXd D;
    //！！！矩阵合并前需要将合并后的矩阵 resize
    D.resize(4, 3);
    D << B.transpose()*B,
        A.transpose();

    Eigen::MatrixXd L3;
    Eigen::MatrixXd One31 = Eigen::MatrixXd::Ones(3, 1);
    L3.resize(4, 1);
    //L3.resize(6, 1);
    L3 << B.transpose()*L2,
        1;
    
    //式（7）
    Eigen::MatrixXd C = (D.transpose()*D).inverse()*D.transpose() * L3;

    //式（8）
    double radius = 0;
    for (int i = 0; i < num; i++)
    {
        Eigen::MatrixXd tmp = M.row(i) - C.transpose();
        radius = radius + sqrt(tmp(0)*tmp(0) + tmp(1)*tmp(1) + tmp(2)*tmp(2));
    }
    radius = radius / num;

    circle.push_back(C(0));
    circle.push_back(C(1));
    circle.push_back(C(2));
    circle.push_back(A(0));
    circle.push_back(A(1));
    circle.push_back(A(2));
    circle.push_back(radius);
    return circle;
}



