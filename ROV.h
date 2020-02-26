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
    float Ix, Iy, Iz;
    float Ixy, Iyx, Ixz, Izx, Iyz, Izy;
    Matrix3f Ib = Matrix3f::Zero(3,3);

    //Center of Gravity
    Vector3f rg;

    //MRB
    Matrix<float,6,6> Mrb;

    //Coeffs. of drag
    float Xu,Yv,Zw,Kp,Mq,Nr;
    float Xuu, Yvv, Zww, Kpp, Mqq, Nrr;
    VectorXf vl = VectorXf::Zero(6);
    VectorXf vnl = VectorXf::Zero(6);

    MatrixXf Dl = MatrixXf::Zero(6,6);
    MatrixXf Dnl = MatrixXf::Zero(6,6);



    static Matrix3f Smtrx(Vector3f r);

public:
    ROV();
    void print_params();
    VectorXf states = VectorXf::Zero(12);
    Matrix<float,6,6> coriolis_matrix(VectorXf cur_state);
    Matrix<float,12,12> A_state_matrix(VectorXf cur_state);
    Matrix<float,12,6> B_state_matrix();

};


#endif //CONTROL_SYSTEM_ROV_H
