/**
 * @file BezierCurve.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-04-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "BezierCurve.h"

/**
 * 阶乘
 * @param n
 * @return
 */
double combination(int n, int k)
{
    double result = 1;
    for (int i = 1; i <= k; ++i)
    {
        result *= static_cast<double>(n - i + 1) / static_cast<double>(i);
    }
    return result;
}

/**
 * 计算n阶贝塞尔曲线上的点
 * @param points 控制点
 * @param t 参数t
 * @return n阶贝塞尔曲线上的点
 */
Vector2d bezier(const vector<Vector2d> &points, double t)
{

    int n = points.size() - 1;
    Vector2d result;
    for (int i = 0; i <= n; ++i)
    {
        result += combination(n, i) * pow(t, i) * pow(1 - t, n - i) * points[i];
    }
    return result;
}