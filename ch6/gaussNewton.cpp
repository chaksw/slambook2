#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
    double ar = 1.0, br = 2.0, cr = 1.0;        // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;       // 估计参数值初始值 x0
    int N = 100;                                // 数据点数量
    double w_sigma = 1.0;                       // 高斯噪声的Sigma(w)方差值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV 随机数产生器

    vector<double> x_data, y_data;              // N个模拟数据 x,y
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);                    // 输入
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));      // 输出，同时另随机数服从正态分布
    }
    // 开始Gauss-Newtion迭代
    int iterations = 100;                       // 迭代100次，直到目标函数趋近收敛
    double cost = 0, lastCost = 0;              // 本次迭代的cost和上一次迭代的cost

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)       // 每一组数据（x,y）都有对应的 H 和 g(这里定义为b:bias)
    {
        Matrix3d H = Matrix3d::Zero();          // Hessian(H) = J^T W^{-1} J in Gauss-Newton
        Vector3d b = Vector3d::Zero();          // bias 偏差
        cost = 0;                               // 增量
        
        // 在每次迭代中，求出最小二乘问题的解error以及Jacobien矩阵J
        for (int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i];   // 定义第i个数据点
            double error = yi - exp(ae * xi * xi + be * xi + ce);   // 从实际参数和输入获得的真实输出与估计输出之间的误差
            Vector3d J;                         // 雅可比矩阵
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);     // de / da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);          // de / db
            J[2] = -exp(ae * xi * xi + be * xi + ce);               // de / dc


            // 增量方程变量
            H += inv_sigma * inv_sigma * J * J.transpose();         // (J * J{-1}) / sigma^2
            b += -inv_sigma * inv_sigma * error * J;                // -J * f(x)(对应这里的error)

            cost += error * error;      // 求解最小二乘问题，这里的error表示估计输出与实际输出的差距，对应无1/2的书式（6.38 
        }
        
        // 求解线性方程 Hx = b
        Vector3d dx = H.ldlt().solve(b);    // 求解稀疏系数矩阵的线性方程组，解dx就是增量
        if (isnan(dx[0]))                   // 判断计算出的数据是否非数值
        {
            cout << "result is nan !" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)       // 根据估计输出与实际输出的误差大小来判断迭代是否得出了足够小的delta[x]
        {
            cout << "cost: " << cost << " >= last cost: " << lastCost << ", break" << endl;
            break;
        }
        // 更新估计值，通过高斯牛顿法，估计值会逐渐逼近实际值，使得误差不断缩小，当增量趋近于零，就可以说算法收敛，实现数据拟合
        ae += dx[0];        
        be += dx[1];
        ce += dx[2];

        lastCost = cost;    // 更新误差，准备与下一次迭代结果比较

        cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
            "\t\testimated params: " << ae << ", " << be << ", " << ce << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}