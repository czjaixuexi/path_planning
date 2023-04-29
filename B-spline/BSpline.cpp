/**
 * @file BSpline.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-04-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "BSpline.h"

/**
 * @brief 基函数定义
 * @param i
 * @param k B样条阶数k
 * @param u 自变量
 * @param knots 节点向量 array([u0,u1,u2,...,u_n+k],shape=[1,n+k+1].
 */
double basic(int i, int k, double u, vector<double> node_vector)
{
    // 0次B样条（1阶B样条）
    double Bik_u;
    if (k == 1)
    {
        if (u >= node_vector[i] && u < node_vector[i + 1])
        {
            Bik_u = 1;
        }
        else
        {
            Bik_u = 0;
        }
    }
    else
    {
        // 公式中的两个分母
        double denominator_1 = node_vector[i + k - 1] - node_vector[i];
        double denominator_2 = node_vector[i + k] - node_vector[i + 1];
        // # 如果遇到分母为 0的情况：
        // # 1. 如果此时分子也为0，约定这一项整体为0；
        // # 2. 如果此时分子不为0，则约定分母为1 。
        if (denominator_1 == 0)
            denominator_1 = 1;
        if (denominator_2 == 0)
            denominator_2 = 1;
        Bik_u = (u - node_vector[i]) / denominator_1 * basic(i, k - 1, u, node_vector) + (node_vector[i + k] - u) / denominator_2 *
                                                                                             basic(i + 1, k - 1, u, node_vector);
    }
    return Bik_u;
}

/**
 * @brief 准均匀B样条的节点向量计算
 * 首末值定义为 0 和 1
 * @param n 控制点n个
 * @param k B样条阶数k， k阶B样条，k-1次曲线.
 * @return vector<double>
 */
vector<double> QuasiUniform(int n, int k)
{
    int numKnots = n + k; // 节点个数
    vector<double> knots(numKnots);

    for (int i = 0; i < numKnots; i++)
    {
        if (i < k)
            knots[i] = 0.0f; // 前k个节点向量为0
        else if (i >= n)
            knots[i] = 1.0f; // 后k个节点向量为1
        else
        {
            // 计算均匀参数值
            knots[i] += knots[i - 1] + (double)1 / (n - k + 1);
        }
    }

    return knots;
}

/**
 * @brief 均匀B样条的节点向量计算
 *
 * @param n 控制点n个
 * @param k B样条阶数k， k阶B样条，k-1次曲线
 * @return vector<double>
 */
vector<double> Uniform(int n, int k)
{
    int numKnots = n + k;
    vector<double> knots(numKnots);

    for (int i = 0; i < numKnots; i++)
    {
        knots[i] = (double)i / (double)(numKnots - 1);
    }

    return knots;
}