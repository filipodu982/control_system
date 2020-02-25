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

    Vector3f rg;

    Matrix<float,6,6> Mrb;






    static Matrix3f Smtrx(Vector3f r);
public:
    void print_params();
    ROV();
};


#endif //CONTROL_SYSTEM_ROV_H
