// To use liberary Eigen to represente Martrix and vector
#include <iostream>
using namespace std;

#include <ctime>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算 （逆、特征值等）
#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

/*************************
* 演示Eigen基本类型的使用
*************************/

int main(int argc, char *argv[]){
	// Eigen 中所有向量和矩阵都是 Eigen::Matrix，它是一个模版类（所以通过声明命名空间来简化代码）。它的前三个参数位：数据类型，行列
	// 声明一个 2*3 的 float 矩阵
	Matrix<float,2,3> matrix_23;
	
	// 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是 Eigen::Matrix
	// 例如 Vector3d 实质上是Eigen::Matrix<double,3,1>,即三维向量
	Vector3d v_3d;
	// 上面的数据声明等价于：(类型不同)
	Matrix<float,3,1> vd_3d;

	// Matrix3d 实质上是 Eigen::Matrix<double,3,3>
	Matrix3d matrix_33 = Matrix3d::Zero();	// 初始化为零
	// 如果不确定矩阵大小，可以使用动态大小的矩阵
	Matrix<double,Dynamic,Dynamic> matric_dynamic;	//动态大小
	// 或者更简单的使用 MatrixXd
	MatrixXd matrix_x;

	// 下面对Eigen阵的操作
	// 输入数据（初始化）
	matrix_23 << 1,2,3,4,5,6;
	// 输出
	cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

	// 用()访问矩阵中的元素
	cout << "print matirx 2x3: " << endl;
	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 3; j++) cout << matrix_23(i,j) << "\t";

	cout << endl;
	}

	// 矩阵和向量相乘（实际上仍是矩阵和矩阵）
	v_3d << 3,2,1;
	vd_3d << 4,5,6;

	// 但是在Eigen里你不能混合两种不同数据类型的矩阵
	// Matrix<double,2,1> resulte_wrong_type = matrix_23 * v_3d; 前者是float 后者是double
	// 应该先进行显示转换 cast<>
	Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
	cout << "[1,2,3;4,5,6*[3,2,1]=" << result.transpose() <<endl;	// 相乘后矩阵是2行1列，用transpose转置为1行两列

	Matrix<float,2,1> result2 = matrix_23 * vd_3d;
	cout << "[1,2,3;4,5,6]*[4,5,6]:" << result2.transpose() << endl;

	// 同样不能搞错矩阵的纬度
	// Eigen::Matrix<double,2,3> result_wrong_demension = matrix_23.cast<double>() * v_3d;

	// 矩阵运算举例
	matrix_33 = Matrix3d::Random();	// 随机数矩阵
	cout << "random matrix: \n" << matrix_33 << endl;
	cout << "transpose : \n"<< matrix_33.transpose() <<endl;	//转置
	cout << "sum: " << matrix_33.sum() << endl;	// 各元素和
	cout << "trace: " << matrix_33.trace() <<endl;	// 迹
	cout << "time 10: \n" << 10 * matrix_33 << endl;	// 数乘
	cout << "invers: \n" << matrix_33.inverse() << endl;	// 逆
	cout << "det: " << matrix_33.determinant() << endl;	// 行列式

	// 特征值
	// 实对称矩阵可以保证对角化成功
	SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
	cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
	cout << "Eigen vector = \n" << eigen_solver.eigenvectors() << endl;

	// 解方程
	// 求解 matrix_NN * x = v_Nd 方程 求解x
	// N的大小在前边的宏定义里（50），它是由随机数产生
	// 直接求逆自然是最直接的，但求逆的运算量大
	
	// 初始化矩阵 matrix_NN 和 v_Nd
	Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
	matrix_NN = matrix_NN * matrix_NN.transpose(); //保证半正定
	Matrix<double, MATRIX_SIZE,1> v_Nd = MatrixXd::Random(MATRIX_SIZE,1);

	clock_t time_stt = clock();	// 计时

	// 求解1:直接求逆 计算量大，计算时间长，速度慢
	Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
	cout << "time of normal inversi is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x.transpose() << endl;

	// 通常同矩阵分解来求，例如QR分解，速度会快很多 colPivHousederQr().solver()
	time_stt = clock();
	x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
	cout << "time of Qr decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x.transpose() << endl;

	// 对于正定矩阵，还可以用cholesky分解来解方程	ldit().solver()
	time_stt = clock();
	x = matrix_NN.ldlt().solve(v_Nd);
	cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x.transpose() << endl; 

	return 0;
}
