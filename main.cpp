#include <iostream>
#include "ROV.h"
#include "Eigen/Dense"
using namespace Eigen;

int main() {
    ROV robot;
    Matrix3d m1 = Matrix3d::Random();
    MatrixXd m(6,6);
    m = MatrixXd::Zero(6,6);
   // std::cout << m << std::endl;
    m.block<3,3>(0,0) = m1;
    //std::cout << m << std::endl;
    Vector3f vec (1.0,2.0,3.0);
    std::cout << robot.Smtrx(vec) << std::endl;
    return 0;
}
