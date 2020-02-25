//
// Created by filip on 25.02.2020.
//
#include <iostream>
#include "Eigen/Dense"
#include "ROV.h"

ROV::ROV(){
    Ix = 0.3; Iy = 0.3; Iz = 0.3;
    Ixy=0; Iyx=0; Ixz=0; Izx=0; Iyz=0; Izy=0;

    Ib << Ix, -Ixy, -Ixz,
            -Iyx, Iy, -Iyz,
            -Izx, -Izy, Iz;


    m = 19;

    rg << 0,0,0.02;

    Mrb.block(0,0,3,3) = m*Matrix3f::Identity(3,3);
    Mrb.block(0,3,3,3) = -m*Smtrx(rg);
    Mrb.block(3,0,3,3) = m*Smtrx(rg);
    Mrb.block(3,3,3,3) = Ib;
}

void ROV::print_params() {
    std::cout << Mrb << std::endl;
}

Matrix3f ROV::Smtrx(Eigen::Vector3f r) {
    Eigen::Matrix3f mtrx;
    mtrx << 0, -r(2), r(1),
    r(2), 0, -r(0),
    -r(1), r(0), 0;
    return mtrx;
}
