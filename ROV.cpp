//
// Created by filip on 25.02.2020.
//
#include <iostream>
#include "Eigen/Dense"
#include "ROV.h"

//Constructor initializing variables
ROV::ROV(){
    init_geometry();
    init_drag();
}


//Function creating a special kind of matrix
//Its definition can be found in Fossen.
Matrix3d ROV::Smtrx(Eigen::Vector3d r) {
    Eigen::Matrix3d mtrx;
    mtrx << 0, -r(2), r(1),
    r(2), 0, -r(0),
    -r(1), r(0), 0;
    return mtrx;
}


//Initializing mass, inertia moments, rg 
void ROV::init_geometry(){
    //Inertia moments
    Ix = 0.3; Iy = 0.3; Iz = 0.3;
    Ixy=0; Iyx=0; Ixz=0; Izx=0; Iyz=0; Izy=0;

    //Moments of inertia matrix
    Ib << Ix, -Ixy, -Ixz,
            -Iyx, Iy, -Iyz,
            -Izx, -Izy, Iz;

    //Mass
    m = 19;
    //Location of CoG in relation to Center of Origin of axes. It is set to 0.02 in z as it is located at Center of Buoy.
    rg << 0,0,0.02;

    Mrb.block(0,0,3,3) = m*Matrix3d::Identity(3,3);
    Mrb.block(0,3,3,3) = -m*Smtrx(rg);
    Mrb.block(3,0,3,3) = m*Smtrx(rg);
    Mrb.block(3,3,3,3) = Ib;

}

//Initializing drag matrices
void ROV::init_drag(){
    //Coeffs. of linear drag
        Xu = -4.03;
        Yv = -6.22;
        Zw = 5.18;
        Kp = -0.07;
        Mq = -0.07;
        Nr = -0.07;

    //Coeffs. of quadratic drag
        Xuu = -18.18;
        Yvv = -21.66;
        Zww = 36.99;
        Kpp = -1.55;
        Mqq = -1.55;
        Nrr = -1.55;

    //Creating diagonal matrices
        vl << Xu,Yv,Zw,Kp,Mq,Nr;
        vnl << Xuu,Yvv,Zww,Kpp,Mqq,Nrr;
        Dl = vl.asDiagonal();
        Dnl = vnl.asDiagonal();
}


//Function for creating Coriolis forces matrix
//Its definition can be found in Fossen
//I've only rewritten it into C++
Matrix<double, 6, 6> ROV::coriolis_matrix(VectorXd cur_state) {
    //Initializing parameters
    Matrix<double,6,6> Crb;    
    Matrix<double,6,1> speed;
    Matrix3d M11, M12, M21, M22;
    Vector3d nu1, nu2;

    //Splitting linear and angular velocities into 2 vectors
    speed = cur_state.block(6,0,6,1);
    nu1 = speed.block(0,0,3,1);
    nu2 = speed.block(3,0,3,1);

    //Creating sub-matrices derived from rigid-body mass matrix
    Mrb = 0.5*(Mrb * Mrb.transpose());  //Making matrix square
    M11 = Mrb.topLeftCorner(3,3);
    M12 = Mrb.topRightCorner(3,3);
    M21 = Mrb.bottomLeftCorner(3,3);
    M22 = Mrb.bottomRightCorner(3,3);

    //Creating Coriolis forces matrix
    Crb << Matrix3d::Zero(3,3), -Smtrx(M11*nu1 + M12*nu2),
            -Smtrx(M11*nu1 + M12*nu2), -Smtrx(M21*nu1 + M22*nu2);

    return Crb;
}


//Function for creating State Space A matrix.
//Its definition, sizes and elements are defined in documentation
Matrix<double, 12, 12> ROV::A_state_matrix(VectorXd cur_state) {
    Matrix<double,12,12> A = MatrixXd::Zero(12,12);
    Matrix<double,6,1> speed = MatrixXd::Zero(6,1);
    Matrix<double,6,6> damping_coeffs = MatrixXd::Zero(6,6);
    MatrixXd speed_diag = MatrixXd::Zero(6,6);

    //Obtaining velocity vector and putting it as diagonal into a speed_diag matrix
    speed = cur_state.block(6,0,6,1);
    speed_diag = speed.asDiagonal();

    //This definition can also be found in documentation
    //First I create damping_coeffs matrix which is the sum
    //Of all elements which create opposing forces
    //Then I divide it by -M matrix which comes from State Space equation
    damping_coeffs = Dnl * speed_diag + coriolis_matrix(cur_state) + Dl;
    damping_coeffs = -Mrb.inverse() * damping_coeffs;

    //State Space matrix
    A << MatrixXd::Zero(6,6), MatrixXd::Zero(6,6),
        MatrixXd::Zero(6,6), damping_coeffs;

    return A;



}

//Function for creating State Space matrix B
//Its definition can also be found in the documentation.
//It can be represented as:
//B = [0
//    1/M]
Matrix<double, 12, 6> ROV::B_state_matrix() {
    Matrix<double, 12,6> B = MatrixXd::Zero(12,6);
    B.block(6,0,6,6) = Mrb.inverse();
    return B;
}
