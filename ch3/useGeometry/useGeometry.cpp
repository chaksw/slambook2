#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// 演示 Eigen 几何模块的使用方法

int main(int argc, char *argv[]){

    // Egein/Geometry 模块提供了各种旋转和平移的表示
    // 两种方法创建一个R（旋转矩阵/向量）
    // 第一种方法：先创建一个单位矩阵、然后通过AngleAxisd模版类（定义旋转的角度和旋转轴）来创建旋转向量，最后用toRotationMatrix()将旋转向量变为旋转矩阵）
    // 3D 旋转矩阵直接使用 Matrix3d 或 Martix3f
    Matrix3d rotation_matrix = Matrix3d::Identity();    // Identity:创建一个单位矩阵
    // 旋转向量使用 AngleAxis, 它底层不直接是 Matrix，但运算可以当作矩阵（因为重载了运算符）
    // AngleAxisd(angle:float, aixs: Vector3) 创建一个定义了旋转角度和旋转轴的旋转向量
    // 第二种方法：直接将创建好的旋转向量用matrix()转化为旋转矩阵
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));    // 沿 Z 轴旋转45度 M_PI = 3.1415926... MPI/4代表 45度 
    cout.precision(3);  // precision:精度控制函数
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl; // 用matrix()转换成矩阵
    // 也可以直接复制
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 用 AngleAxis 可以进行坐标转换
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // 注：欧拉角是用来确定定点转动刚体位置的3个一组独立角参量，由yaw（绕Z轴旋转的偏航角） pitch（绕Y轴旋转的俯仰角） roll（绕X轴旋转的滚转角）组成
    // 欧拉角：可以将旋转矩阵直接转换成欧拉角，通过旋转矩阵中已知的旋转角和旋转轴，求能实现相同旋转的欧拉角三个角参量的值
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);   // 函数内参数定义了旋转了顺序，这里为ZYX顺序，即yaw-pitch-roll顺序
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 创建欧氏变换矩阵（T）使用 Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();      // //()内参数可以是旋转矩阵/向量，也可以是四元数，虽然为3D，实质上是4*4的矩阵
    T.rotate(rotation_vector);      // 按照rotation_vector进行旋转
    T.pretranslate(Vector3d(1, 3, 4));  // 把平移向量设成（1，3，4）
    cout << "Trasform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v; // 相当于 R*v + t
    cout << "v transformed = " << v_transformed.transpose() << endl;

    // 彷射（Ts）和射影变换（Tp）和欧式变换（T）相似（实质上是4*4的矩阵）
    // 对于彷射和射影变换，使用 Eigen::Affine3d 和 Eigen::Prijective3d 替换 Eigen::Isometry3d 即可，略

    // 四元数
    // 可以直接把AngleAxis复制给四元数，反之亦然 q0 = cos(delta/2) [q1,q2,q3] = [n1,n2,n3]sin(delta/2) 其中[n1,n2,n3]为旋转轴
    Quaterniond q = Quaterniond(rotation_vector);   
    // q.coeffs() 表示四元数的四个参数[q0,q1,q2,q3]
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;  // 请注意coeffs的顺序是（x,y,z,w）,w为实部，前三者为虚部
    // 也可以把旋转矩阵赋给它
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;  
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q * v; // 注意数学上是qvq^{-1}
    cout << " (1,0,0) after rotation of quaternion= " << v_rotated.transpose() << endl;
    // 用常规向量乘法表示，则应该如下计算 v' = q*v*q^{-1} 其中v(1,0,0)的四元数格式为(0,1,0,0)得出的结果也是四元数
    cout << "should be equal to " << (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs().transpose() << endl;

    return 0;


}