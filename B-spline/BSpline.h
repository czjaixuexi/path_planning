/**
 * @file BSpline.h
 * @author czj
 * @brief 
 * @version 0.1
 * @date 2023-04-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BSPLINE_H
#define BSPLINE_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace Eigen;

double basic(int i, int k, double u, vector<double> knots);

vector<double> QuasiUniform(int n, int k);

vector<double> Uniform(int n, int k);

#endif // BSPLINE_H
