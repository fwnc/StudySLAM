#include <iostream>
using namespace std;
#include <ctime>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
using namespace Eigen;
int main() {
    //动态定义100X100矩阵
    Matrix<double, Dynamic, Dynamic> matrix_100X100;
    matrix_100X100 = MatrixXd::Random(100,100);
    matrix_100X100 = matrix_100X100 * matrix_100X100.transpose();  // 保证半正定
    //动态定义100X1矩阵
    Matrix<double, Dynamic, 1> B = MatrixXd::Random(100, 1);

    // 直接求逆
    clock_t time_stt = clock(); // 计时
    Matrix<double, Dynamic, 1> x = matrix_100X100.inverse() * B;
    cout << "time of normal inverse is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;


    // QR分解
    time_stt = clock();
    x = matrix_100X100.colPivHouseholderQr().solve(B);
    cout << "time of Qr decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    // 对于正定矩阵，还可以用cholesky分解来解方程
    time_stt = clock();
    x = matrix_100X100.ldlt().solve(B);
    cout << "time of ldlt decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;



    return 0;
}
