#ifndef MATH_H
#define MATH_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>


Eigen::MatrixXd  computePeseudoInverse(const Eigen::MatrixXd& jacobian);


#endif // !MATH_H
