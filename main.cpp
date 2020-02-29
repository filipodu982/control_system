#include <iostream>
#include "ROV.h"
#include "Eigen/Core"
#include <ct/optcon/optcon.h>

#include "Eigen/Dense"
using namespace Eigen;

int main() {
    //Class constructor
    ROV robot;

    //Sizes of state and control vectors
    //ROV has 12 states [6-DOF position, velocities]
    //And 6 control as it has 6-DOF
    const size_t state_dim = 12;
    const size_t control_dim = 6;

    ct::optcon::LQR<state_dim,control_dim> lqrSolver;   //Initializing lqr solver
    
    Matrix<double,6,12> K;                              //K matrix which corresponds to gain of regulator
    VectorXd v = VectorXd::Zero(12);                    //Test state vector

    Matrix<double,state_dim,state_dim> A;               //A state space matrix
    Matrix<double,state_dim,control_dim> B;             //B state space matrix

    A = robot.A_state_matrix(v);
    B = robot.B_state_matrix();

    //Q and R are weight matrices of LQR. For testing they are assigned as random values 
    MatrixXd Q = MatrixXd::Random(12,12);
    MatrixXd R = MatrixXd::Random(6,6);

    //Computing K matrix
    lqrSolver.compute(Q,R,A,B,K);
    std::cout<<"Macierz K\n"<<K<<std::endl;
    return 0;
}
