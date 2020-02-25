//
// Created by filip on 25.02.2020.
//
#include <iostream>
#include "ROV.h"

void ROV::print_params() {
    std::cout << m << D << std::endl;
}

Matrix3f ROV::Smtrx(Eigen::Vector3f r) {
    Eigen::Matrix3f mtrx;
    mtrx << 0, -r(2), r(1),
    r(2), 0, -r(0),
    -r(1), r(0), 0;
    return mtrx;
}
