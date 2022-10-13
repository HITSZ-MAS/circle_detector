#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

vector<double> FitCircle(std::vector<vector<double>> pts)
{
    vector<double> circle;

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
    L3 << B.transpose()*L2,
        One31;
    
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


int main()
{

//     M=[0.306400001049,7.283800125122,-0.833899974823
// 0.303600013256,7.299799919128,-0.830200016499
// 0.330099999905,7.292799949646,-0.831799983978
// 0.312299996614,7.306300163269,-0.830500006676
// 0.317299991846,7.278500080109,-0.838299989700
// 0.325800001621,7.303400039673,-0.829900026321
// 0.326700001955,7.281099796295,-0.831200003624
// 0.302300006151,7.289100170135,-0.829999983311
// 0.311199992895,7.279699802399,-0.837000012398
// 0.319200009108,7.305699825287,-0.829500019550
// 0.307599991560,7.302400112152,-0.829599976540
// 0.319700002670,7.279099941254,-0.838800013065

// ];

    std::vector<std::vector<double>>  pts={{0.306400001049,7.283800125122,-0.833899974823},
                                                                                {0.303600013256,7.299799919128,-0.830200016499},
                                                                                {0.330099999905,7.292799949646,-0.831799983978},
                                                                                {0.312299996614,7.306300163269,-0.830500006676},
                                                                                {0.317299991846,7.278500080109,-0.838299989700},
                                                                                {0.325800001621,7.303400039673,-0.829900026321},
                                                                                {0.326700001955,7.281099796295,-0.831200003624},
                                                                                {0.302300006151,7.289100170135,-0.829999983311},
                                                                                {0.311199992895,7.279699802399,-0.837000012398},
                                                                                {0.319200009108,7.305699825287,-0.829500019550},
                                                                                {0.319700002670,7.279099941254,-0.838800013065},
                                                                                {0.319700002670,7.279099941254,-0.838800013065}};
    std::vector<double> circle;
    circle = FitCircle(pts);

    for(int i=0;i<circle.size();i++)
    {
        cout<<circle[i]<<endl;
    }
    return 0;
}