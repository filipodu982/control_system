#include <iostream>
#include "ROV.h"
#include "Eigen/Core"
#include <ct/optcon/optcon.h>
#include <cmath>
#include <EigenQP.h>
#include "Eigen/Dense"
#include <chrono>
using namespace Eigen;

int main() {
    //Class constructor
    ROV robot;

//    int n,m,p;
//    n = 10;
//    m = 5;
//    p = 7;
//
//    MatrixXd eG = MatrixXd::Random(n,n);
//    MatrixXd eCi = MatrixXd::Random(n,p);
//    MatrixXd eCe = MatrixXd::Random(n,m);
//    VectorXd eg0 = VectorXd::Random(n);
//    VectorXd ece = VectorXd::Random(m);
//    VectorXd eci = VectorXd::Random(p);
//    VectorXd ex = VectorXd::Random(n);
//
//    eG = (eG.transpose() * eG).eval();
//    std::cout << QP::solve_quadprog(eG,eg0,eCe,ece,eCi,eci,ex) << std::endl;
//    std::cout << "dupa " << ex << std::endl;



    //Sizes of state and control vectors
    //ROV has 12 states [6-DOF position, velocities]
    //And 6 control as it has 6-DOF
    const size_t state_dim = 12;
    const size_t control_dim = 6;

    ct::optcon::LQR<state_dim,control_dim> lqrSolver;   //Initializing lqr solver
    
    Matrix<double,6,12> K;                              //K matrix which corresponds to gain of regulator
    VectorXd v = VectorXd::Zero(12);               //Test state vector

    Matrix<double,state_dim,state_dim> A;               //A state space matrix
    Matrix<double,state_dim,control_dim> B;             //B state space matrix

    A = robot.A_state_matrix(v);
    B = robot.B_state_matrix();

    //Q and R are weight matrices of LQR. For testing they are assigned as random values 
    MatrixXd Q = MatrixXd::Random(12,12);
    MatrixXd R = MatrixXd::Random(6,6);

    //Computing K matrix
    lqrSolver.compute(Q,R,A,B,K);

    //Desired tau for testing
    VectorXd t(6);
    t<<1,2,9,0.0,0.0,0.0;

    //Loop for calculating QP and seeing the change in angle and calculating the time it takes
    auto start = std::chrono::high_resolution_clock::now();
    for(int i =0; i <= 500; i++){
        robot.thrust_allocation(t);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

    std::cout<<"Duration: " << duration.count() << std::endl;


    //std::cout<<"Macierz K\n"<<K<<std::endl;
    return 0;
}
