//
// Created by filip on 25.02.2020.
//

#ifndef CONTROL_SYSTEM_ROV_H
#define CONTROL_SYSTEM_ROV_H

#include "Eigen/Dense"
using namespace Eigen;

class ROV {
private:

    // Mass and inertia moments
    int m;
    double Ix, Iy, Iz;
    double Ixy, Iyx, Ixz, Izx, Iyz, Izy;
    Matrix3d Ib = Matrix3d::Zero(3,3);

    //Center of Gravity
    Vector3d rg;

    //MRB
    Matrix<double,6,6> Mrb;

    //Coeffs. of drag
    double Xu,Yv,Zw,Kp,Mq,Nr;
    double Xuu, Yvv, Zww, Kpp, Mqq, Nrr;
    VectorXd vl = VectorXd::Zero(6);
    VectorXd vnl = VectorXd::Zero(6);
    
    //Diagonal matrices of coeffs
    MatrixXd Dl = MatrixXd::Zero(6,6);
    MatrixXd Dnl = MatrixXd::Zero(6,6);



    static Matrix3d Smtrx(Vector3d r);  //Function creating a special kind of matrix
    void init_geometry();               //Initializing mass, inertia moments, rg
    void init_drag();                   //initializing drag matrices 

public:
    ROV();                                                   //Constructor initializing variables
   // VectorXd states = VectorXd::Zero(12);
    Matrix<double,6,6> coriolis_matrix(VectorXd cur_state);
    Matrix<double,12,12> A_state_matrix(VectorXd cur_state);
    Matrix<double,12,6> B_state_matrix();

};


#endif //CONTROL_SYSTEM_ROV_H
