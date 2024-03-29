#ifndef QPSOLVER_H
#define QPSOLVER_H

// osqp 所需头文件 osqp is C language.
#include <iostream>
#include <osqp.h>
#include <stdlib.h>
#include <stdio.h>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>

Eigen::MatrixXd qpSolver(const Eigen::MatrixXd& p, const Eigen::MatrixXd& q, const Eigen::MatrixXd& a,
	const Eigen::MatrixXd& l, const Eigen::MatrixXd& u, const int n = 2, const int m = 3);


#endif