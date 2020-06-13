/* 例子：
    设有小萝卜1号和二号位于世界坐标系中。记世界坐标系为 W，小萝卜们的坐标系为 R1 和 R2。
    小萝卜1号位姿为q1=[0.35,0.2,0.3,0.1]， t1 = [0.3,0.1,0.1]。小萝卜2号的位姿为
    q2 = [-0.5,0.4,-0.1,0.2], t2 = [-0.1,0.5,0.3]。这里的q和t表达的是TRk,w,k = 1,2
    也就是世界坐标系到相机坐标系的变换关系。现在，小萝卜1号看到某个点在自身的坐标系下坐标为
    pr1 = [0.5,0,0.2]，求该向量在小萝卜2号坐标系下的坐标 */

#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char* argv[]){

    Quaterniond q1(0.35,0.2,0.3,0.1),q2(-0.5,0.4,-0.1,0.2);
    // 注意：四元数使用之前需要归一化：normalize()
    q1.normalize();
    q2.normalize();
    Vector3d t1(0.3,0.1,0.1),t2(-0.1,0.5,0.3);
    Vector3d pr1(0.5,0,0.2);

    Isometry3d T1w(q1),T2w(q2);    // 将四元数转化为变换矩阵T 
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);   // 添加平移向量 t1,t2

    /* 计算 pr2: T1w 和 T2w 分别描述了小萝卜1号和小萝卜2号从世界坐标系到相机坐标系的变换
        对于pr1，假设它在世界坐标下的坐标为 p, 则有 pr1 = T1w * p，同理可得p在小萝卜二号坐标系中的坐标为：
        pr2 = T2w * p，可得：pr2 = T2w * T1w^{-1} * pr1 */
    Vector3d pr2 = T2w * T1w.inverse() * pr1;
    cout << endl << pr2.transpose() << endl;

    return 0;
}

