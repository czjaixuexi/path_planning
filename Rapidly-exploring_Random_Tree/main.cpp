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
#include "RRT.h"

int main()
{
    vector<vector<double>> obstacle_list{
        {5, 5, 1},
        {3, 6, 2},
        {3, 8, 2},
        {3, 10, 2},
        {7, 5, 2},
        {9, 5, 2},
        {8, 10, 1},
        {6, 12, 1}};
    Node *begin = new Node(0.0, 0.0);
    Node *end = new Node(6.0, 10.0);
    vector<double> rnd_area{-2, 15};         // 采样区域 x,y ∈ [min,max];
    vector<double> play_area{-2, 12, 0, 14}; // 约束随机树的范围 [xmin,xmax,ymin,ymax]
    double radius = 0.8;                     // 机器人半径
    double expand_dis = 2;                   // 扩展的步长
    double goal_sample_rate = 5;             // 采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
    int max_iter = 500;                      // 最大采样点数
    RRT rrt(obstacle_list, rnd_area, play_area, radius, expand_dis, goal_sample_rate, max_iter);
    rrt.setBegin(begin);
    rrt.setEnd(end);

    pair<vector<double>, vector<double>> traj = rrt.planning();

    plt::plot(traj.first, traj.second, "r");

    // // 合成 GIF 图片
    // stringstream filename;
    // filename << "./frame_" << gifindex << ".png";
    // plt::save(filename.str());

    // const char *gif_filename = "./rrt_demo.gif";
    // stringstream cmd;
    // cmd << "convert -delay 10 -loop 0 ./frame_*.png " << gif_filename;
    // system(cmd.str().c_str());
    // cout << "Saving result to " << gif_filename << std::endl;
    // plt::show();
    // //删除png图片
    // system("rm *.png");
    // return 0;

    const char *filename = "./rrt_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}