#include <iostream>
#include "Eigen/Dense"
#include "ROV.h"
#include <ct/optcon/optcon.h>
#include <cmath>
#include <EigenQP.h>
#include <Eigen/src/Core/Matrix.h>

#include <chrono>
//using namespace Eigen;

int main() {
    //Class constructor
    ROV robot;

    //Sizes of state and control vectors
    //controlSystem has 12 states [6-DOF position, velocities]
    //And 6 control as it has 6-DOF
    const size_t stateDim = 12;
    const size_t controlDim = 6;

    ct::optcon::LQR<stateDim,controlDim> lqrSolver;   //Initializing lqr solver

    Eigen::Matrix<double,6,12> K;                              //K matrix which corresponds to gain of regulator
    Eigen::VectorXd v = Eigen::VectorXd::Zero(12);               //Test state vector

    Eigen::Matrix<double,stateDim,stateDim> A;               //A state space matrix
    Eigen::Matrix<double,stateDim,controlDim> B;             //B state space matrix

    std::cout<<"Zainicializowani"<<std::endl;

    A = robot.A_state_matrix(v);
    B = robot.B_state_matrix();

    //Q and R are weight matrices of LQR. For testing they are assigned as random values
//    Eigen::MatrixXd Q = Eigen::MatrixXd::Random(12,12);
//    Eigen::MatrixXd R = Eigen::MatrixXd::Random(6,6);

    Eigen::Matrix<double, stateDim,stateDim> Q = Eigen::Matrix<double, stateDim,stateDim>::Zero(12,12);
    Eigen::Matrix<double, controlDim,controlDim> R = Eigen::Matrix<double, controlDim,controlDim>::Zero(6,6);

    Eigen::Matrix<double,stateDim,stateDim> Qhelp = Eigen::Matrix<double,stateDim,stateDim>::Zero();
    Eigen::Matrix<double,controlDim,controlDim> Rhelp = Eigen::Matrix<double,controlDim,controlDim>::Zero();

    Eigen::VectorXd Qdiag(12);
    Eigen::VectorXd Rdiag(6);


    Qhelp.setOnes(12,12);
    Rhelp.setOnes(6,6);

    Qdiag << 500, 500, 1500, 5000, 5000, 5000, 1, 1, 1, 18000, 18000, 18000;
    Rdiag << 1,1,0.5,1,1,1;


    Q = Qdiag.asDiagonal();
    R = Rdiag.asDiagonal();


    std::cout<<A<<std::endl<<std::endl<<B<<std::endl;
    //Computing K matrix
    lqrSolver.compute(Q,R,A,B,K,true, true);

    std::cout<<K<<std::endl;

    Eigen::VectorXd pos(12);
    pos << 1,0,0,0,0,0,0,0,0,0,0,0;

    Eigen::VectorXd desiredPos(6);
    Eigen::MatrixXd error(6,1);

    error = desiredPos - (K*pos);

    std::cout<<"Blad\n"<<error.transpose()<<std::endl;



    //Desired tau for testing
    Eigen::VectorXd t(6);
    t<<1,2,9,0.0,0.0,0.0;

    //Loop for calculating QP and seeing the change in angle and calculating the time it takes
//    auto start = std::chrono::high_resolution_clock::now();
//    for(int i =0; i <= 500; i++){
//        robot.thrustAllocation(t);
//    }
//    auto stop = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
//
//    std::cout<<"Duration: " << duration.count() << std::endl;


    //std::cout<<"Macierz K\n"<<K<<std::endl;
    return 0;
}
