#include <iostream>
#include "ROV.h"
#include "Eigen/Core"
#include <ct/optcon/optcon.h>

#include "Eigen/Dense"
using namespace Eigen;

int main() {
    ROV robot;
    const size_t state_dim = 12;
    const size_t control_dim = 6;

    ct::optcon::LQR<state_dim,control_dim> lqrSolver;
    Matrix<double,6,12> K;
    Matrix3d m1 = Matrix3d::Random();
    MatrixXd m = MatrixXd::Random(6,6);
    VectorXd v = VectorXd::Zero(12);

    Matrix<double,12,12> A;
    Matrix<double,12,6> B;
    //m = MatrixXd::Zero(6,6);
   // std::cout << m << std::endl;
    //m.block<3,3>(0,0) = m1;
    //robot.print_params();
    A = robot.A_state_matrix(v);
    B = robot.B_state_matrix();
    MatrixXd Q = MatrixXd::Random(12,12);
    MatrixXd R = MatrixXd::Random(6,6);

    lqrSolver.compute(Q,R,A,B,K);
    std::cout<<"Macierz K\n"<<K<<std::endl;
    return 0;
}
