/**
 * @file main.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-04-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "BSpline.h"
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main()
{
    // 控制点
    vector<Vector2d> points{Vector2d(0, 0), Vector2d(1, 1), Vector2d(2, 1), Vector2d(3, 1), Vector2d(4, 2)};

    // 生成控制点
    vector<double> x(points.size()), y(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        x[i] = points[i][0];
        y[i] = points[i][1];
    }

    // 生成曲线点
    vector<double> x_, y_;
    int n = points.size(); // 控制点个数
    int k = 3;             // k阶、k-1次B样条
    Vector2d p_u(0.0, 0.0);
    vector<double> b(n);

    int flag;
    cout << "请选择：1. 均匀B样条 2.准均匀B样条 0. 退出 " << endl;
    cin >> flag;
    vector<double> knots;
    switch (flag)
    {
    case 1: // 均匀B样条
        knots = Uniform(n, k);
        for (double u = (double)(k - 1) / (n + k); u < (double)(n + 1) / (n + k); u += 0.005)
        {
            plt::clf();
            for (int i = 0; i < n; i++)
            {
                b[i] = basic(i, k, u, knots);
                // cout << b[i] << endl;
            }
            for (int i = 0; i < points.size(); i++)
            {
                p_u += points[i] * b[i];
            }
            x_.push_back(p_u[0]);
            y_.push_back(p_u[1]);
            // 画图
            // plt::xlim(0,1);
            plt::plot(x_, y_, "r");
            plt::plot(x, y);
            plt::pause(0.01);
            p_u = Vector2d(0, 0);
        }
        break;
    case 2: // 准均匀B样条
        knots = QuasiUniform(n, k);
        for (double u = 0; u < 1; u += 0.005)
        {
            plt::clf();
            for (int i = 0; i < n; i++)
            {
                b[i] = basic(i, k, u, knots);
                // cout << b[i] << endl;
            }
            for (int i = 0; i < points.size(); i++)
            {
                p_u += points[i] * b[i];
                // cout<<p_u<<","<<endl;
            }
            x_.push_back(p_u[0]);
            y_.push_back(p_u[1]);
            // 画图
            // plt::xlim(0,1);
            plt::plot(x_, y_, "r");
            plt::plot(x, y);

            // // 将每一帧保存为单独的文件
            // stringstream filename;
            // filename << "./frame_" << u * 1000 << ".png";
            // plt::save(filename.str());
            plt::pause(0.01);
            p_u = Vector2d(0, 0);
        }
        break;
    default:
        return 0;
    }

    // // 合成 GIF 图片
    // const char *gif_filename = "./b_spline_demo.gif";
    // stringstream cmd;
    // cmd << "convert -delay 10 -loop 0 ./frame_*.png " << gif_filename;
    // system(cmd.str().c_str());
    // cout << "Saving result to " << gif_filename << std::endl;
    // plt::show();
    // // 删除png图片
    // system("rm *.png");
    // return 0;

    // save figure
    const char *filename = "./b_spline_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}