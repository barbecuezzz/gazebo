#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include <Eigen/Dense>
#include<iostream>
#include"build_robot.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double,6,6> Matrix6d;

Matrix6d CalcJacobian(ListNode *from,ListNode *to);
Matrix3d FindRoute(ListNode *from,ListNode *to);
Matrix3d Rodrigues(Vector3d a, double q);











#endif