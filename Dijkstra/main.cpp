/**
 * @file main.cpp
 * @author czj
 * @brief 主函数
 * @version 0.1
 * @date 2023-04-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Dijkstra.h"

int main()
{
    vector<double> start{-5, -5}, goal{50, 50};
    double grid_size = 2.0;
    double robot_radius = 1.0;

    vector<double> ox;
    vector<double> oy;

    Dijkstra dijkstra(grid_size, robot_radius);
    dijkstra.setObstale(ox, oy);       // 设置障碍物信息
    dijkstra.set(start, goal, ox, oy); // 设置起点，目标点，障碍物的(x,y)
    dijkstra.calObstacleMap(ox, oy);   // 在地图上生成障碍物
    dijkstra.getMotionModel();         // 创建移动代价

    // 绘制地图
    plt::plot(ox, oy, ".k");
    plt::plot(vector<double>{start[0]}, vector<double>{start[1]}, "ob");
    plt::plot(vector<double>{goal[0]}, vector<double>{goal[1]}, "or");
    plt::grid(true);

    // 规划路径
    pair<vector<double>, vector<double>> xy = dijkstra.planning(start, goal);
    // 绘制路径
    plt::plot(xy.first, xy.second, "-r");

    // // 合成 GIF 图片
    // stringstream filename;
    // filename << "./frame_" << gifindex << ".png";
    // plt::save(filename.str());
    // const char *gif_filename = "./dijkstra_demo.gif";
    // stringstream cmd;
    // cmd << "convert -delay 3 -loop 0 ./frame_*.png " << gif_filename;
    // system(cmd.str().c_str());
    // cout << "Saving result to " << gif_filename << std::endl;
    // plt::show();
    // //删除png图片
    // system("rm *.png");
    // return 0;

    // 保存图片
    const char *filename = "./dijkstra_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();

    return 0;
}
