/**
 * @file Dijkstra.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-04-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Dijkstra.h"

//int gifindex = 0;

Dijkstra::Node::Node(double x, double y, float cost, double parentIndex) : x(x), y(y), cost(cost), parent_index(parentIndex) {}

Dijkstra::Dijkstra(double resolution, double robotRadius) : resolution(resolution), robot_radius(robotRadius) {}

/**
 * @brief 设置障碍物以及边缘
 *
 */
void Dijkstra::setObstale(vector<double> &ox, vector<double> &oy)
{
    for (double i = -10; i < 60; i++)
    {
        ox.push_back(i);
        oy.push_back(-10.0);
    }
    for (double i = -10; i < 60; i++)
    {
        ox.push_back(60.0);
        oy.push_back(i);
    }
    for (double i = -10; i < 61; i++)
    {
        ox.push_back(i);
        oy.push_back(60.0);
    }
    for (double i = -10; i < 61; i++)
    {
        ox.push_back(-10.0);
        oy.push_back(i);
    }
    for (double i = -10; i < 10; i++)
    {
        ox.push_back(i);
        oy.push_back(10);
    }
    for (double i = 0; i < 30; i++)
    {
        ox.push_back(40.0 - i);
        oy.push_back(30);
    }
    for (double i = 0; i < 15; i++)
    {
        ox.push_back(60.0 - i);
        oy.push_back(30);
    }
}

/**
 * @brief 得到障碍物信息图，有障碍物的地方标记为true，没有标记为false
 * @param ox 障碍物x坐标集合
 * @param oy 障碍物y坐标集合
 */
void Dijkstra::calObstacleMap(const vector<double> &ox, const vector<double> &oy)
{
    min_x = round(*min_element(ox.begin(), ox.end()));
    min_y = round(*min_element(oy.begin(), oy.end()));
    max_x = round(*max_element(ox.begin(), ox.end()));
    max_y = round(*max_element(oy.begin(), oy.end()));

    cout << "min_x:" << min_x << "   min_y:" << min_y << "  max_x:" << max_x << "  max_y:" << max_y << endl;

    x_width = round((max_x - min_x) / resolution);
    y_width = round((max_y - min_y) / resolution);
    cout << "x_width:" << x_width << "  y_width:" << y_width << endl;

    obstacle_map = vector<vector<bool>>(x_width, vector<bool>(y_width, false));

    for (double i = 0; i < x_width; i++)
    {
        double x = calPosition(i, min_x);
        for (double j = 0; j < y_width; j++)
        {
            double y = calPosition(j, min_y);
            for (double k = 0; k < ox.size(); k++)
            {
                double d = sqrt(pow(ox[k] - x, 2) + pow(oy[k] - y, 2));
                if (d <= robot_radius)
                {
                    obstacle_map[i][j] = true;
                    break;
                }
            }
        }
    }
}

/**
 * @brief 计算栅格在地图中的位置
 * @param index
 * @param minp
 * @return
 */
double Dijkstra::calPosition(double index, double minp)
{
    double pos = index * resolution + minp;
    return pos;
}

/**
 * @brief 标记移动代价
 * @return
 */
void Dijkstra::getMotionModel()
{
    // x,y,cost
    motion = {{1, 0, 1},
              {0, 1, 1},
              {-1, 0, 1},
              {0, -1, 1},
              {-1, -1, sqrt(2)},
              {-1, 1, sqrt(2)},
              {1, -1, sqrt(2)},
              {1, 1, sqrt(2)}};
}

/**
 * @brief 计算起点终点的栅格索引
 * @param position
 * @param minp
 * @return
 */
double Dijkstra::calXyIndex(double position, double minp)
{
    return round((position - minp) / resolution);
}

/**
 * @brief 计算栅格索引
 * @param node
 * @return
 */
double Dijkstra::calIndex(Dijkstra::Node *node)
{
    // cout<<node->x<<","<<node->y<<endl;
    return (node->y - min_y) * x_width + (node->x - min_x);
}

/**
 * @brief 判断节点是否有效，即是否超出边界和碰到障碍物
 * @param node
 * @return
 */
bool Dijkstra::verifyNode(Dijkstra::Node *node)
{
    double px = calPosition(node->x, min_x);
    double py = calPosition(node->y, min_y);
    // 超出边界
    if (px < min_x || py < min_y || px >= max_x || py >= max_y)
        return false;
    // 遇到障碍物
    if (obstacle_map[node->x][node->y])
        return false;
    return true;
}

/**
 * @brief 根据parent_index，倒推出路径
 * @param goal_node
 * @param closed_set
 * @return
 */
pair<vector<double>, vector<double>> Dijkstra::calFinalPath(Dijkstra::Node *goal_node, map<double, Node *> closed_set)
{
    vector<double> rx, ry;
    rx.push_back(calPosition(goal_node->x, min_x));
    ry.push_back(calPosition(goal_node->y, min_y));

    double parent_index = goal_node->parent_index;

    while (parent_index != -1)
    {
        Node *node = closed_set[parent_index];
        rx.push_back(calPosition(node->x, min_x));
        ry.push_back(calPosition(node->y, min_y));
        parent_index = node->parent_index;
    }
    return {rx, ry};
}

/**
 * @brief 规划
 * @param start 起点
 * @param goal 终点
 * @return 规划后的路径
 */
pair<vector<double>, vector<double>> Dijkstra::planning(const vector<double> start, const vector<double> goal)
{
    double sx = start[0], sy = start[1];
    double gx = goal[0], gy = goal[1];
    Node *start_node = new Node(calXyIndex(sx, min_x), calXyIndex(sy, min_y), 0.0, -1);
    Node *goal_node = new Node(calXyIndex(gx, min_x), calXyIndex(gy, min_y), 0.0, -1);

    map<double, Node *> open_set, closed_set;
    // 将起点加入到open set
    open_set[calIndex(start_node)] = start_node;

    Node *current = nullptr;
    while (true)
    {
        double cur_id = numeric_limits<double>::max();
        double cost = numeric_limits<double>::max();
        // 计算open_set中代价最小的节点（此处可以用优先队列优化）
        for (auto it = open_set.begin(); it != open_set.end(); it++)
        {
            if (it->second->cost < cost)
            {
                cost = it->second->cost;
                cur_id = it->first; // index
            }
        }
        current = open_set[cur_id];

        plotGraph(current); // 画图

        // 若找到了目标结点，则退出循环
        if (abs(current->x - goal_node->x) < EPS && abs(current->y - goal_node->y) < EPS)
        {
            cout << "Find goal" << endl;
            goal_node->parent_index = current->parent_index;
            goal_node->cost = current->cost;
            break;
        }

        // 从open set中去除
        auto iter = open_set.find(cur_id);
        open_set.erase(iter);
        // 将其加入到closed set
        closed_set[cur_id] = current;

        // 根据motion，扩展搜索网络
        for (vector<double> move : motion)
        {
            // cout<<move[0]<<move[1]<<move[2]<<endl;
            // 根据motion，计算相邻结点的cost，然后创建该节点
            Node *node = new Node(current->x + move[0], current->y + move[1], current->cost + move[2], cur_id);
            double n_id = calIndex(node);
            // 如果已经在closed_set中了
            if (closed_set.find(n_id) != closed_set.end())
                continue;
            // 如果超出边界或者碰到障碍物了
            if (!verifyNode(node))
                continue;
            // 如果open set中没有这个节点
            if (open_set.find(n_id) == open_set.end())
            {
                open_set[n_id] = node;
            }
            // 如果open set中已经存在这个节点,则更新cost
            else
            {
                if (open_set[n_id]->cost >= node->cost)
                {
                    open_set[n_id] = node;
                }
            }
        }
    }
    return calFinalPath(goal_node, closed_set);
}

/**
 * @brief 画图
 * @param current
 */
void Dijkstra::plotGraph(Dijkstra::Node *current)
{
    plt::plot(vector<double>{calPosition(current->x, min_x)}, vector<double>{calPosition(current->y, min_y)}, "xc");
    // // 将每一帧保存为单独的文件
    // stringstream filename;
    // filename << "./frame_" << gifindex++ << ".png";
    // plt::save(filename.str());
    plt::pause(0.0000001);
}

/**
 * @brief 设置坐标
 *
 * @param st 起点
 * @param go 目标
 * @param ox 障碍物x
 * @param oy 障碍物y
 */
void Dijkstra::set(const vector<double> &st, const vector<double> &go, const vector<double> &ox, const vector<double> &oy)
{
    Dijkstra::st = st;
    Dijkstra::go = go;
    Dijkstra::ox = ox;
    Dijkstra::oy = oy;
}
