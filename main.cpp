#include <iostream>
#include "ROV.h"
#include "Eigen/Dense"
using namespace Eigen;

int main() {
    ROV robot;
    Matrix3d m1 = Matrix3d::Random();
    MatrixXd m = MatrixXd::Random(6,6);
    VectorXd v = VectorXd::Zero(12);
    //m = MatrixXd::Zero(6,6);
   // std::cout << m << std::endl;
    //m.block<3,3>(0,0) = m1;
    //robot.print_params();
    robot.B_state_matrix();
    //std::cout << robot.Smtrx(vec) << std::endl;
    return 0;
}
