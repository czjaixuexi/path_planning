# 简介

自动驾驶常用路径规划算法C++实现



# 项目依赖

推荐在Ubuntu 18.04/20.04 环境下运行

- **cmake**

  在Ubuntu中安装cmake：

  ```
  sudo apt install cmake
  ```

- **Eigen**

  在Ubuntu中安装Eigen：

  ```
  sudo apt-get install libeigen3-dev
  ```

- **python3**



# 编译

在当前目录下输入：

```shell
mkdir build
cd build
cmake ../
make
```



# Path_planning

## Dijkstra



![dijkstra_demo](README.assets/dijkstra_demo.gif)

## A star

![astar](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/astar-168329807331712.gif)

## RRT



![rrt_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/rrt_demo-168329807983514.gif)



## RRT connect

![rrt_connect](README.assets/rrt_connect-16832995877726.gif)

## RRT star

![rrt_star_demo](README.assets/rrt_star_demo.gif)



## Bezier

![bezier_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/bezier_demo.gif)



## B spline

![b_spline_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/b_spline_demo.gif)





# 参考

[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics#pythonrobotics)

[chhRobotics_CPP](https://github.com/CHH3213/chhRobotics_CPP)

