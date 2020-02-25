#include <iostream>
#include "Eigen/Dense"
using namespace Eigen;

int main() {
    Vector4d x(3,4,5,6);
    std::cout<<x<<std::endl;
    DiagonalMatrix<double,4> m;
    m.diagonal() = x;
    std::cout << m.toDenseMatrix() << std::endl;
    return 0;
}
