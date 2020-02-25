//
// Created by filip on 25.02.2020.
//

#ifndef CONTROL_SYSTEM_ROV_H
#define CONTROL_SYSTEM_ROV_H

#include "Eigen/Dense"
using namespace Eigen;

class ROV {
private:
    int m = 40;
    float D = 0.3;

public:
    Matrix3f Smtrx(Vector3f r);
    void print_params();
};


#endif //CONTROL_SYSTEM_ROV_H
