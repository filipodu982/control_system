//
// Created by filip on 25.02.2020.
//
#include <iostream>
#include "Eigen/Dense"
#include "ROV.h"

ROV::ROV(){
    //Inertia moments
    Ix = 0.3; Iy = 0.3; Iz = 0.3;
    Ixy=0; Iyx=0; Ixz=0; Izx=0; Iyz=0; Izy=0;

    Ib << Ix, -Ixy, -Ixz,
            -Iyx, Iy, -Iyz,
            -Izx, -Izy, Iz;

    //Mass
    m = 19;
    //Location of CoG in relation to Center of Origin of axes. It is set to 0.02 in z as it is located at Center of Buoy.
    rg << 0,0,0.02;

    Mrb.block(0,0,3,3) = m*Matrix3f::Identity(3,3);
    Mrb.block(0,3,3,3) = -m*Smtrx(rg);
    Mrb.block(3,0,3,3) = m*Smtrx(rg);
    Mrb.block(3,3,3,3) = Ib;

    //Coeffs. of drag
        Xu = -4.03;
        Yv = -6.22;
        Zw = 5.18;
        Kp = -0.07;
        Mq = -0.07;
        Nr = -0.07;

        Xuu = -18.18;
        Yvv = -21.66;
        Zww = 36.99;
        Kpp = -1.55;
        Mqq = -1.55;
        Nrr = -1.55;

        vl << Xu,Yv,Zw,Kp,Mq,Nr;
        vnl << Xuu,Yvv,Zww,Kpp,Mqq,Nrr;
        Dl = vl.asDiagonal();
        Dnl = vnl.asDiagonal();
}



void ROV::print_params() {
    std::cout << Dl << std::endl;
}

Matrix3f ROV::Smtrx(Eigen::Vector3f r) {
    Eigen::Matrix3f mtrx;
    mtrx << 0, -r(2), r(1),
    r(2), 0, -r(0),
    -r(1), r(0), 0;
    return mtrx;
}

Matrix<float, 6, 6> ROV::coriolis_matrix(VectorXf cur_state, Matrix<float, 6, 6> mass_mtrx) {
    Matrix<float,6,6> Crb;
    Matrix<float,6,1> speed;
    Matrix3f M11, M12, M21, M22;
    Vector3f nu1, nu2;

    speed = cur_state.block(6,0,6,1);
    nu1 = speed.block(0,0,3,1);
    nu2 = speed.block(3,0,3,1);

    mass_mtrx = 0.5*(mass_mtrx * mass_mtrx.transpose());
    M11 = mass_mtrx.topLeftCorner(3,3);
    M12 = mass_mtrx.topRightCorner(3,3);
    M21 = mass_mtrx.bottomLeftCorner(3,3);
    M22 = mass_mtrx.bottomRightCorner(3,3);

    Crb << Matrix3f::Zero(3,3), -Smtrx(M11*nu1 + M12*nu2),
            -Smtrx(M11*nu1 + M12*nu2), -Smtrx(M21*nu1 + M22*nu2);
    std::cout<<speed<<std::endl;



    return Crb;
}
