#include "competition1/ColorDetect.h"

#include <iostream>

ColorDetector::ColorDetector()
{
    fx=320.0;
    fy=320.0;
    cx=320.0;
    cy=240.0;
    realRadius = 1.02;
    circle_num_ = 0;
    PoseX_ = 0.0;
    PoseY_ = 0.0;
    PoseZ_ = 0.0;
    PoseYaw_ = 0.0;

    depth_fx_ = 160.0 *2;
    depth_fy_ = 160.0* 2;
    depth_cx_ = 160.0*2;
    depth_cy_ = 120.0*2;
}

ColorDetector::ColorDetector(ros::NodeHandle &nh)
{
    nh_=nh;
    config_ = new DetectorConfig(nh_);
    Initialize();
}

void ColorDetector::Initialize()
{
    fx=config_->color_fx_;
    fy=config_->color_fy_;
    cx=config_->color_cx_;
    cy=config_->color_cy_;

    color_height_ = config_->color_height_;
    color_width_ = config_->color_width_;
    depth_height_=config_->depth_height_;
    depth_width_ = config_->depth_width_;

    realRadius = config_->real_circle_radius_big_;
    circle_num_ = 0;
    PoseX_ = 0.0;
    PoseY_ = 0.0;
    PoseZ_ = 0.0;
    PoseYaw_ = 0.0;

    is_resize_depth_ = config_->resize_depth_;
    if(is_resize_depth_)
    {
        depth_fx_ = config_->depth_fx_ *2;
        depth_fy_ = config_->depth_fy_* 2;
        depth_cx_ = config_->depth_cx_*2;
        depth_cy_ = config_->depth_cy_*2;
    }
    else
    {
        depth_fx_ = config_->depth_fx_;
        depth_fy_ = config_->depth_fy_;
        depth_cx_ = config_->depth_cx_;
        depth_cy_ = config_->depth_cy_;
    }

    Rfc_.SetParams(200,0.95,0.07);

    std::cout<<color_height_<<std::endl;
    std::cout<<"Initialize successfully!!!!!!!!!!!!!!!!!"<<std::endl;
}


/**
 * @brief  Detect the desired color and return a binary map of the corresponding color area
 * 
 * @param inputImg color image
 * @param colortype the color we want to detect
 * @param outputImg binary image of the color area
 * @return true 
 * @return false 
 */
bool ColorDetector::Detect(cv::Mat inputImg , int colortype , cv::Mat &outputImg)
{
    // std::vector<cv::Mat>channels;
	// cv::split(inputImg, channels);

	// cv::Mat blue, green, red;
	// blue = channels.at(0);
	// green = channels.at(1);
	// red = channels.at(2);
	// // 分别对BGR通道做直方图均衡化
	// cv::equalizeHist(blue, blue);
	// cv::equalizeHist(green, green);
	// cv::equalizeHist(red, red);
    // cv::merge(channels, inputImg);
	// cv::imshow("zhifangtujunheng", inputImg);

    if(inputImg.empty())
    {
        std::cout<<"empty mat"<<std::endl;
        return false;
    }
    else
    {
        cv::Mat HSVimg,temp,temp2;
        inputImg = inputImg(cv::Range(1,400),cv::Range(50,590));
        cv::GaussianBlur(inputImg, inputImg, cv::Size(3, 3),1, 1);
        cv::cvtColor(inputImg, HSVimg, CV_BGR2HSV);
        cv::inRange(HSVimg, cv::Scalar(COLOR[colortype][0], COLOR[colortype][2], COLOR[colortype][4]), 
                                                    cv::Scalar(COLOR[colortype][1], COLOR[colortype][3], COLOR[colortype][5]),temp); //Threshold the image
        //cv::inRange(HSVimg, cv::Scalar(0, COLOR[colortype][2], COLOR[colortype][4]), 
        //                                            cv::Scalar(10, COLOR[colortype][3], COLOR[colortype][5]),temp2); //Threshold the image
        //temp = temp+temp2;
        cv::Mat output;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        cv::morphologyEx(temp,output,cv::MORPH_OPEN,kernel);
        outputImg = output.clone();
        // cv::imshow("color" , outputImg);
        return true;
    }
}


/**
 * @brief  Hough Line Transform function
 * 
 * @param image 
 */
void ColorDetector::HoughTranslate(cv::Mat& image)
{
	//做边缘检测首先要进行高斯边缘模糊，如果利用Canny高斯边缘模糊Canny会帮我们做
	cv::Mat image_gray, dst;
	cv::Canny(image, image_gray, 150, 200);//100,200分别是低阈值和高阈值输出二值图
	// cv::imshow("edge_image", image_gray);
	cv::cvtColor(image_gray, dst, cv::COLOR_GRAY2BGR);
 
	std::vector<cv::Vec4f> plines;//吧每个像素点的平面坐标转化为极坐标产生的曲线放入集合中
	cv::HoughLinesP(image_gray, plines, 1, CV_PI / 180.0,100, 100,10);//从平面坐标转换到霍夫空间,最终输出的是直线的两个点（x0,y0,x1,y1）
    //cv::HoughLinesP(,)
    std::cout<<"line number:"<<plines.size()<<std::endl;
	cv::Scalar color = cv::Scalar(0, 0, 255);
	for (size_t i = 0; i < plines.size(); i++)
	{
		cv::Vec4f hline = plines[i];
		line(dst, cv::Point(hline[0], hline[1]), cv::Point(hline[2], hline[3]), color, 3, cv::LINE_AA);
	}
	// imshow("hough_line_detection", dst);
 
}


/**
 * @brief this function for draw the biggest contours , return the biggest rect
 *                in this function : image m is the final image we want
 * 
 * @param inputImg binary image
 * @param boundRect Maximum outline circumscribed rectangle
 * @return true have rect
 * @return false no rect 
 */
bool ColorDetector::DrawRect(cv::Mat inputImg,cv::Rect &boundRect)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy2;
    cv::findContours(inputImg,contours,hierarchy2,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    cv::Mat m = cv::Mat::zeros(inputImg.size(), CV_8UC1); //最小外接矩形画布  

    std::vector<double> g_dConArea(contours.size());

    //
    if(contours.size()<=0)
    {
        //no contours
        return false;
    }
    //

    for (int i = 0; i < contours.size(); i++)
    {
        //计算轮廓的面积
        g_dConArea[i] = cv::contourArea(contours[i]);
    }
    //寻找面积最大的部分
    int max = 0;
    for (int i = 1; i < contours.size(); i++) 
    {
        if (g_dConArea[i] > g_dConArea[max]) 
        {
            max = i;
        }
    }

    //
    if(g_dConArea[max]<200)
    {
        //so small
        return false;
    }
    if(g_dConArea[max]>40000)
    {
        //so big
        circle_num_++;
        return false;
    }
    //
    
    //ji suan zhou chang
    double arc_length = cv::arcLength(contours[max],true);

    //绘制轮廓
    //TODO: use image m to do hough circle
    //cv::Rect boundRect;
    cv::drawContours(m, contours, max, cv::Scalar(255), 0.5, 4, hierarchy2);
    boundRect = cv::boundingRect(cv::Mat(contours[max]));
    rectangle(m, cv::Point(boundRect.x, boundRect.y), cv::Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), cv::Scalar(255), 1, 8);

    double d = (boundRect.width+boundRect.height)/2;
    double arc_circle = d*3.1416;
    if(abs(arc_length - arc_circle)>d/4)
    {
        std::cout<<"not a circle"<<std::endl;
        return false;
    }
    if(abs(boundRect.width-boundRect.height)>30)
    {
        std::cout<<"not a circle"<<std::endl;
        return false;
    }

    //cv::namedWindow("标注出circle", cv::WINDOW_NORMAL);
    //cv::imshow("标注出circle",inputImg );

    return true;
}


/**
 * @brief use Hough Circle Transform
 *              this function is deprecated
 * 
 * @param inputImg gray image
 * @param circles circle we detected
 * @return true circle exsits
 * @return false circle does not exsit
 */
bool ColorDetector::DrawCircle(cv::Mat inputImg , std::vector<cv::Vec3f> &circles)
{
    cv::HoughCircles(inputImg,circles,CV_HOUGH_GRADIENT,2,100,200,100,1,500);

    if(circles.size()<=0)
    {
        //no circles detected
        return false;
    }
    std::cout<<circles.size()<<std::endl;

    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆心
        cv::circle(inputImg, center, 3, cv::Scalar(44, 25, 32), -1, 8, 0);
        //绘制圆轮廓
        cv::circle(inputImg, center, radius, cv::Scalar(155, 50, 255), 3, 8, 0);
    }
    // cv::namedWindow("标注出circle", cv::WINDOW_NORMAL);
    // cv::imshow("标注出circle",inputImg );

    return true;
}


/**
 * @brief use Hough Circle Transform
 * 
 * @param input gray image
 * @param circles circle we detected
 * @return true circle exsits
 * @return false circle does not exsit
 */
bool ColorDetector::HoughFindCircle(cv::Mat& input , std::vector<cv::Vec3f> &circles)
{
    //to gray
    cv::Mat gray;
    //input = input(cv::Range(100,350),cv::Range(80,460));
    //cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(input, input, cv::Size(5, 5),2, 2);
    // cv::imshow("gray",gray);
	HoughCircles(input, circles, cv::HOUGH_GRADIENT,1.8,300,200,100,10,300);

    if(circles.size()<=0)
    {
        //no circles
        return false;
    }
	//依次在途中绘制出圆
    cv::Mat drawimage = input.clone();
    cv::cvtColor(input,drawimage,cv::COLOR_GRAY2RGB);
    std::cout<<circles.size()<<std::endl;
	for (size_t i = 0; i < circles.size(); i++)
	{
		//参数定义
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		//绘制圆心
		circle(drawimage, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		//绘制圆轮廓
		circle(drawimage, center, radius, cv::Scalar(155, 20, 255), 3, 8, 0);
	}
    //cv::namedWindow("标注出circle", cv::WINDOW_NORMAL);
    //cv::imshow("标注出circle",drawimage );

    return true;
}


/**
 * @brief 3D point estimation of the center of the circle using the detected largest rectangular outline
 *              this function is deprecated
 * 
 * @param rect 
 */
void ColorDetector::CirclePose(cv::Rect &rect)
{
    double centerx,centery;
    centerx =rect.x+rect.width/2;
    centery=rect.y+rect.height/2;

    if(rect.height-rect.width<20)
    {
        //we consider this is a circle with yaw = 0
        double r = double((rect.height+rect.width))/2.0/2.0;

        //delta u = fx *delta X / Z
        double z = 1.0*fx*realRadius/r;
        z=Z_From_Depth_;
        //[u,v,1]^T = K*[X/Z,Y/Z,1]^T
        double poseXc=z*(double(centerx-cx))/fx;
        double poseYc=z*(double(centery-cy))/fy;
        double poseZc=z;
        double yaw=0.0;
        PoseX_ = poseZc+0.26;
        PoseY_ = -poseXc;
        PoseZ_ = -poseYc;
        std::cout<<"x:"<<PoseX_<<",y:"<<PoseY_<<",z:"<<PoseZ_<<std::endl;
    }
    else
    {
        //TODO yaw isn't 0.0
        double scale=0.0;
        double r , z;
        if(rect.height>rect.width)
        {
            scale = double(rect.width)/double(rect.height);
            r = double(rect.height) /2.0;
        }
        else
        {
            scale = double(rect.height)/double(rect.width);
            r = double(rect.width) /2.0;
        }
        
        double yaw = acos(scale);
        z = 1.0*fy*realRadius/r;
        //[u,v,1]^T = K*[X/Z,Y/Z,1]^T
        double poseXc=z*(double(centerx-cx))/fx;
        double poseYc=z*(double(centery-cy))/fy;
        double poseZc=z;
        PoseX_ = poseZc+0.26;
        PoseY_ = -poseXc;
        PoseZ_ = -poseYc;
        std::cout<<"x:"<<PoseX_<<",y:"<<PoseY_<<",z:"<<PoseZ_<<std::endl;
    }
    Circles_Pose_.clear();
    std::vector<double> p(4);
    p[0]=PoseX_;
    p[1]=PoseY_;
    p[2]=PoseZ_;
    p[3]=0.0;
    Circles_Pose_.push_back(p);
}


/**
 * @brief 3D point estimation of the center of the circle using the detected largest rectangular outline
 *              this function is deprecated
 * @param rects 
 */
void ColorDetector::CirclePose(std::vector<cv::Rect> &rects)
{
    Circles_Pose_.clear();
    for(int i=0 ; i<rects.size() ; ++i)
    {
        double centerx,centery;
        centerx =rects[i].x+rects[i].width/2;
        centery=rects[i].y+rects[i].height/2;
        if(rects[i].height-rects[i].width<20)
        {
            //we consider this is a circle with yaw = 0
            double r = double((rects[i].height+rects[i].width))/2.0/2.0;

            //delta u = fx *delta X / Z
            double z = 1.0*fx*realRadius/r;
            z=Z_From_Depth_;
            //[u,v,1]^T = K*[X/Z,Y/Z,1]^T
            double poseXc=z*(double(centerx-cx))/fx;
            double poseYc=z*(double(centery-cy))/fy;
            double poseZc=z;
            double yaw=0.0;
            PoseX_ = poseZc+0.26;
            PoseY_ = -poseXc;
            PoseZ_ = -poseYc;
            std::cout<<"x:"<<PoseX_<<",y:"<<PoseY_<<",z:"<<PoseZ_<<std::endl;

            std::vector<double> p(4);
            p[0]=PoseX_;
            p[1]=PoseY_;
            p[2]=PoseZ_;
            p[3]=0.0;
            Circles_Pose_.push_back(p);
        }
    }
}


/**
 * @brief 3D point estimation of the center of the circle using the detected 2D circular contour
 *              this function is deprecated
 * 
 * @param circles 
 * @param DETECTTYPE 
 */
void ColorDetector::CirclePose(std::vector<cv::Vec3f> circles , int DETECTTYPE)
{
    if(circles.size()<=0)
    {
        //no circles
        return;
    }

    if(DETECTTYPE == ColorDetector::COLOR_TYPE)
    {
        int circlenum = circles.size();
        for(int i=0;i<circlenum;i++)
        {
            double r=circles[i][2];
            double centerx = circles[i][0];
            double centery = circles[i][1];

            //delta u = fx *delta X / Z
            double z = 1.0*fx*realRadius/r;

            //[u,v,1]^T = K*[X/Z,Y/Z,1]^T
            double poseXc=z*(double(centerx-cx))/fx;
            double poseYc=z*(double(centery-cy))/fy;
            double poseZc=z;
            double yaw=0.0;

            PoseX_ = poseZc+0.26;
            PoseY_ = -poseXc;
            PoseZ_ = -poseYc;
            std::cout<<"x:"<<PoseX_<<",y:"<<PoseY_<<",z:"<<PoseZ_<<std::endl;

            //TODO 
            //isInSquare();
        }
    }
    else if(DETECTTYPE == ColorDetector::DEPTH_TYPE)
    {
        Circles_Pose_.clear();
        int circlenum = circles.size();
        for(int i=0;i<circlenum;i++)
        {
            double r=circles[i][2];
            double centerx = circles[i][0];
            double centery = circles[i][1];

            //delta u = fx *delta X / Z
            double z = 1.0*depth_fx_*realRadius/r;
            if(z > 10)
            {
                 z = 1.0*depth_fx_*(realRadius+0.1)/r;
            }

            //[u,v,1]^T = K*[X/Z,Y/Z,1]^T
            double poseXc=z*(double(centerx-depth_cx_))/depth_fx_;
            double poseYc=z*(double(centery-depth_cy_))/depth_fy_;
            double poseZc=z;
            double yaw=0.0;

            PoseX_ = poseZc+0.26;
            PoseY_ = -poseXc;
            PoseZ_ = -poseYc;
            std::cout<<"x:"<<PoseX_<<",y:"<<PoseY_<<",z:"<<PoseZ_<<std::endl;
            std::vector<double> p(4);
            p[0]=PoseX_;
            p[1]=PoseY_;
            p[2]=PoseZ_;
            p[3]=0.0;//no yaw calc
            Circles_Pose_.push_back(p);
            //TODO 
            //isInSquare();
        }
    }

}


/**
 * @brief get circle pose once
 *              this function is deprecated
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param yaw 
 */
void ColorDetector::GetCirclePose(double &x,double &y , double &z , double &yaw)
{
    x = PoseX_;
    y = PoseY_;
    z = PoseZ_;
    yaw = PoseYaw_;
}


/**
 * @brief get all (probably) circles pose 
 * 
 * @param p 
 */
void ColorDetector::GetCirclePose(std::vector<std::vector<double>> &p)
{
    p = Circles_Pose_;
}


bool ColorDetector::isInSquare(double centerx,double centery)
{

}


/**
 * @brief pixel coordinate -> camera coordinate
 * 
 * @param x pixel x
 * @param y pixel y
 * @param depth depth measured by depth image
 * @return std::vector<double> points in camrea frame
 */
std::vector<double> ColorDetector::Pixel2Camera(double x,double y,double depth)
{
    std::vector<double> pc(3);
    double poseXc=depth*(double(x-depth_cx_))/depth_fx_;
    double poseYc=depth*(double(y-depth_cy_))/depth_fy_;
    double poseZc=depth;
    double yaw=0.0;

    // PoseX_ = poseZc+0.26;
    // PoseY_ = -poseXc;
    // PoseZ_ = -poseYc;
    pc[0]=poseZc +0.26;
    pc[1]=-poseXc;
    pc[2]=-poseYc;

    return pc;
}


/**
 * @brief  The Hough circle transform is performed on the basis of the depth map to obtain the contour of the target area (circle).
 *  Convert the contour points to the camera coordinate system for storage, to prepare for the subsequent circle fitting
 * 
 * @param inputImg Depth map transformed to grayscale
 * @param boundRect Maximum circumscribed rectangle outline
 * @param depth depth image (float)
 * @return true 
 * @return false 
 */
bool ColorDetector::FindContoursDepth(cv::Mat inputImg, std::vector<cv::Rect> &boundRect , cv::Mat depth)
{
    boundRect.clear();
    Fitted_Circles_.clear();
    Circles_Pose_.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy2;
    cv::findContours(inputImg,contours,hierarchy2,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    cv::Mat m = cv::Mat::zeros(inputImg.size(), CV_8UC1); //最小外接矩形画布  

    std::vector<double> g_dConArea(contours.size());

    //
    int contoursNum = contours.size();
    if(contoursNum<=0)
    {
        //no contours
        return false;
    }

    bool goodRect = false;
    m=inputImg.clone();
    cv::cvtColor(m,m,cv::COLOR_GRAY2RGB);
    for(int i=0; i < contoursNum ; ++i)
    {
        double arc_length=cv::arcLength(contours[i],true);
        double area = cv::contourArea(contours[i]);

        if(area<200)
        {
            //so small
            continue;
        }

        double arc_r = arc_length/2.0/3.1416;
        double area_r = sqrt(area/3.1416);

        if(abs(arc_r-area_r)>17)
        //if(0.9>(arc_r/area_r)||(arc_r/area_r)>1.1)
        {
            //no circle
            continue;
        }
        cv::drawContours(m,contours,i,cv::Scalar(128,0,255),3);

        std::vector<double> circle;
        double depthsum = 0.0;
        int count=0;
        int direct[4][2]={{0,1},{0,-1},{1,0},{-1,0}};
        Circles_Points_Camera_.clear();

        for(int j  = 0; j<contours[i].size();++j)
        {
            cv::Point p = contours[i][j];
            double zz = depth.at<float>(p.y/2,p.x/2);
            if(abs(zz)<0.00001)
            {
                //if(IsInRange(p.x/2,p.y/2))
                //{
                    for(int k=0;k<4;k++)
                    {
                        int row=p.y/2+direct[k][0];
                        int col=p.x/2+direct[k][1];
                        if(IsOutOfRange(depth_width_,depth_height_,row,col))
                        {
                            //Out of Range 
                            continue;
                        }

                        zz = depth.at<float>(row,col);
                        if(abs(zz)>0.0001)
                        {
                            break;
                        }
                    }
               // }
            }
            if(abs(zz)<0.0001)
            {
                continue;
            }
            count++;
            depthsum+=zz;

            //For fit Circle
            std::vector<double> pc;
            pc=Pixel2Camera(p.x,p.y,zz);
            Circles_Points_Camera_.push_back(pc);

        }

        bool isCircle = false;

        if(Circles_Points_Camera_.size()>100)
        {
            //circle = FitCircle(Circles_Points_Camera_);
            isCircle = Rfc_.SolveRANSACFitCircle(Circles_Points_Camera_,circle);
            Fitted_Circles_.push_back(circle);

            ROS_INFO("fit circle successfully!!!!!");

            if(circle[6]>0.9&&circle[6]<1.1)
            {
                std::vector<double> c(3);
                c[0]=circle[0];
                c[1]=circle[1];
                c[2]=circle[2];
                Circles_Pose_.push_back(c);
            }
        }
        else
        {
            ROS_WARN("pts in camera frame isn't enough for detecting!!!");
        }

        Z_From_Depth_ = depthsum/count/2.0;

        if(isCircle)
        {
            cv::Rect boundRectsingle = cv::boundingRect(cv::Mat(contours[i]));
            rectangle(m, cv::Point(boundRectsingle.x, boundRectsingle.y), 
            cv::Point(boundRectsingle.x + boundRectsingle.width, boundRectsingle.y + boundRectsingle.height), cv::Scalar(255), 3, 8);
            boundRect.push_back(boundRectsingle);
        }
        
        goodRect = true;
    }

    if(config_->show_image_)
    {
        cv::imshow("m",m);
    }
    return goodRect;
    
}



/**
 * @brief circle fitting function. Use 3D camera frame points to fit 3D circle
 * 
 * @param pts 3D points in camera frame (vector)
 * @return std::vector<double> : circle [0~6]: center*3 , norm vector*3 , radius*1 . all together : 7
 */
std::vector<double> ColorDetector::FitCircle(std::vector<std::vector<double>> pts)
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
    //std::cout<<"fit circle successfully"<<std::endl;
    ROS_WARN("r = %f" , radius);
    return circle;
}

bool ColorDetector::IsOutOfRange(int width , int height , int row ,int col)
{
    // if(row<0||col<0)
    // {
    //     return true;
    // }

    // if(row>=height||col>=width)
    // {
    //     return true;
    // }

    // return false;
    return (row<0||col<0)||(row>=height||col>=width);
}



