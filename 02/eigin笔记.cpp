"第一次总结"

//矩阵的定义
Matrix<float, 2, 3> matrix_23;
Vector3d = Eigen::Matrix<double, 3, 1>
Matrix<double, Dynamic, Dynamic> matrix_dynamic;    // 如果不确定矩阵大小，可以使用动态大小的矩阵

// 四则运算就不演示了，直接用+-*/即可。
matrix_33 = Matrix3d::Random();      // 随机数矩阵
cout << "random matrix: \n" << matrix_33 << endl;
cout << "transpose: \n" << matrix_33.transpose() << endl;      // 转置
cout << "sum: " << matrix_33.sum() << endl;            // 各元素和
cout << "trace: " << matrix_33.trace() << endl;          // 迹
cout << "times 10: \n" << 10 * matrix_33 << endl;               // 数乘
cout << "inverse: \n" << matrix_33.inverse() << endl;        // 逆
cout << "det: " << matrix_33.determinant() << endl;    // 行列式

// 特征值
// 实对称矩阵可以保证对角化成功
SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;       //特征值
cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;     //特征向量

// QR分解
time_stt = clock();
Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
cout << "time of Qr decomposition is "<< 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
cout << "x = " << x.transpose() << endl;
// 对于正定矩阵，还可以用cholesky分解来解方程
time_stt = clock();
x = matrix_NN.ldlt().solve(v_Nd);
cout << "time of ldlt decomposition is "<< 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
cout << "x = " << x.transpose() << endl;




"第二次总结"

//旋转矩阵的定义
Matrix3d rotation_matrix = Matrix3d::Identity();//用单位矩阵对变量进行了初始化

//旋转向量的定义
AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
//旋转向量--->旋转矩阵
rotation_matrix = rotation_vector.toRotationMatrix();
rotation_vector.matrix()

//旋转矩阵--->欧拉角
Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即roll pitch yaw顺序

// 四元数
Quaterniond q = Quaterniond(rotation_vector);  // 可以直接把AngleAxis赋值给四元数，反之亦然
q = Quaterniond(rotation_matrix);    // 也可以把旋转矩阵赋给它
q.normalize();  //四元数使用前需要进行归一化
cout << "quaternion from rotation vector = " << q.coeffs().transpose()<< endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

// 欧氏变换矩阵的定义
Isometry3d T = Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
T.rotate(rotation_vector);                            // 按照rotation_vector进行旋转
T.pretranslate(Vector3d(1, 3, 4));                     // 把平移向量设成(1,3,4)

// 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略



