#include<iostream>
#include <typeinfo>
#include<Eigen/Core>
#include<Eigen/Dense>

#define MATRIX_SIZE 50

using namespace std;
using namespace Eigen;


int main() {
    // // <float, 2, 3>
    // Matrix<float, 2, 3> matrix_23f;
    // matrix_23f << 1, 2, 3, 4, 5, 6;
    // cout << "= matrix_23f =\n" << matrix_23f << "\n" << endl;

    // // <double, 3, 1>
    // Vector3d v_3d;
    // v_3d << 3, 2, 1;
    // cout << "= v_3d =\n" << v_3d << "\n"  << endl;

    // // <double, 3, 3>
    // Matrix3d matrix_33d = Matrix3d::Zero();
    // cout << "= matrix_33d =\n" << matrix_33d << "\n"  << endl;

    // // cast the data type
    // Matrix<double, 2, 3> res = matrix_23f.cast<double>() * matrix_33d;
    // cout << "= matrix_23f * matrix_33d =\n" << res << "\n"  << endl;

    // <float, 3, 3>
    Matrix3f matrix_33f;
    matrix_33f << 1, 0, 0, 0, 5, 6, 7, 8, 9;
    cout << "= matrix_33f =\n" << matrix_33f << "\n"  << endl;
    cout << "= matrix_33f.transpose() =\n" << matrix_33f.transpose() << "\n"  << endl;
    cout << "= matrix_33f.trace() =\n" << matrix_33f.trace() << "\n"  << endl;
    cout << "= matrix_33f.determinant() =\n" << matrix_33f.determinant() << "\n"  << endl;
    cout << "= matrix_33f.inverse() =\n" << matrix_33f.inverse() << "\n"  << endl;
    cout << "= matrix_33f * matrix_33f.inverse() =\n" << matrix_33f * matrix_33f.inverse() << "\n"  << endl;


    return 0;
}
