cmake_minimum_required(VERSION 3.21)
project(path_planning)

set(CMAKE_CXX_STANDARD 14)


# include(GNUInstallDirs)
set(PACKAGE_NAME path_planning)

# output
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Library target
add_library(path_planning INTERFACE)

# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
target_link_libraries(path_planning INTERFACE
        Python::Python
        Python::Module
        Python::NumPy
        )
install(
        TARGETS path_planning
        EXPORT install_targets
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# bezier
add_executable(bezier_demo Bezier/main.cpp Bezier/BezierCurve.cpp)
target_link_libraries(bezier_demo PRIVATE path_planning)

# B-spline
add_executable(b_spline_demo B-spline/main.cpp B-spline/BSpline.cpp)
target_link_libraries(b_spline_demo PRIVATE path_planning)


# RRT
add_executable(rrt_demo Rapidly-exploring_Random_Tree/main.cpp Rapidly-exploring_Random_Tree/RRT.cpp)
target_link_libraries(rrt_demo PRIVATE path_planning)


# RRT_connect
add_executable(rrt_connect_demo Rapidly-exploring_Random_Tree_connect/main.cpp Rapidly-exploring_Random_Tree_connect/RRT_connect.cpp)
target_link_libraries(rrt_connect_demo path_planning)

# RRT_star
add_executable(rrt_star_demo Rapidly-exploring_Random_Tree_Star/main.cpp Rapidly-exploring_Random_Tree_Star/RRT_Star.cpp Rapidly-exploring_Random_Tree/RRT.cpp)
target_link_libraries(rrt_star_demo path_planning)

# Dijkstra
add_executable(dijkstra_demo Dijkstra/main.cpp Dijkstra/Dijkstra.cpp)
target_link_libraries(dijkstra_demo path_planning)

# astar
add_executable(astar_demo A_Star/Astar.cpp A_Star/main.cpp)
target_link_libraries(astar_demo path_planning)