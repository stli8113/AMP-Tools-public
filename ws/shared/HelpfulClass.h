#pragma once

#include "AMPCore.h"

namespace amp{

Eigen::Matrix2d MyBugAlgorithm::rotationMatrix(double theta){
    Eigen::MatrixXd m(2,2);
    m(0,0) = std::cos(theta);
    m(1,0) = std::sin(theta);
    m(0,1) = -std::sin(theta);
    m(1,1) = std::cos(theta);
    // std::cout << "theta: "<< theta << "\n matrix:" <<  m << std::endl;
    return m;
}

}