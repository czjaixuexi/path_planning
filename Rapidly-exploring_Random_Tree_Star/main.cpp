//
// Created by chh3213 on 2022/11/27.
//
#include "RRT_Star.h"

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
    vector<double> rnd_area{-2, 15};
    vector<double> play_area{-2, 12, 0, 14};
    double radius = 0.5;
    double expand_dis = 1;        // 扩展的步长
    double goal_sample_rate = 20; // 采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
    int max_iter = 500;
    double connect_circle_dist = 5.0;
    bool search_until_max_iter = false;

    RRT_Star rrt(obstacle_list, rnd_area, play_area, radius, expand_dis, goal_sample_rate, max_iter, connect_circle_dist, search_until_max_iter);
    rrt.setBegin(begin);
    rrt.setEnd(end);

    pair<vector<double>, vector<double>> traj = rrt.planning();

    plt::plot(traj.first, traj.second, "r");

    //     // 合成 GIF 图片
    // stringstream filename;
    // filename << "./frame_" << gifindex << ".png";
    // plt::save(filename.str());

    // const char *gif_filename = "./rrt_star_demo.gif";
    // stringstream cmd;
    // cmd << "convert -delay 10 -loop 0 ./frame_*.png " << gif_filename;
    // system(cmd.str().c_str());
    // cout << "Saving result to " << gif_filename << std::endl;
    // plt::show();
    // //删除png图片
    // system("rm *.png");
    // return 0;

    // save figure
    const char *filename = "./rrt_star_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}
