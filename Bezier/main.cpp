/**
 * @file main.cpp
 * @author czj
 * @brief 
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "BezierCurve.h"
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main()
{
    // 控制点
    vector<Vector2d> points{Vector2d(0, 0), Vector2d(1, 1), Vector2d(2.5, 1.5), Vector2d(3, 1), Vector2d(4, 2)};

    // 生成控制点
    cout << "Plotting..." << endl;
    vector<double> x(points.size()), y(points.size());
    for (int i = 0; i < points.size(); ++i)
    {
        x[i] = points[i][0];
        y[i] = points[i][1];
    }

    // 生成贝塞尔曲线
    const int n_points = 100;
    vector<Vector2d> curve(n_points);
    vector<double> x_curve, y_curve;
    for (int i = 0; i < n_points; ++i)
    {
        plt::clf();
        double t = static_cast<double>(i) / n_points;
        curve[i] = bezier(points, t);
        x_curve.emplace_back(curve[i][0]);
        y_curve.emplace_back(curve[i][1]);
        // 绘制控制点和曲线
        plt::plot(x_curve, y_curve, "r");
        plt::plot(x, y);
        // // 将每一帧保存为单独的文件
        // stringstream filename;
        // filename << "./frame_" << i << ".png";
        // plt::save(filename.str());
        plt::pause(0.01);
    }
    // // 合成 GIF 图片
    // const char *gif_filename = "./bezier_demo.gif";
    // stringstream cmd;
    // cmd << "convert -delay 10 -loop 0 ./frame_*.png " << gif_filename;
    // system(cmd.str().c_str());
    // cout << "Saving result to " << gif_filename << std::endl;
    // plt::show();
    // //删除png图片
    // system("rm *.png");
    // return 0;


    // save figure
    const char* filename = "./bezier_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}