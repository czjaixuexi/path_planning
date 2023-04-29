/**
 * @file BezierCurve.h
 * @author czj
 * @brief 
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace Eigen;

double combination(int n, int k);

Vector2d bezier(const vector<Vector2d>& points, double t);


#endif // BEZIERCURVE_H
