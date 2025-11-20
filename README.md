# Grid Based Planning Tutorial

Publisher: Taehyun Jung

E-mail: jth8090@khu.ac.kr

This repository contains all source code and lecture materials used in my tutorial series on **Grid-Based Planning**.

The goal of this project is to bridge three layers:

1. **Theory** – grids, occupancy grids, costmaps, inflation, classical search algorithms
2. **Standalone algorithms** – pure Python implementations with step-by-step visualization on Windows
3. **ROS 2 integration** – applying the same ideas to real `OccupancyGrid` maps in Gazebo + RViz

Everything here is organized so that you can:

- watch the lecture videos,
- open the slides,
- run the same code on Windows or on Ubuntu/ROS 2,
- and clearly see how the theory is used in practice.

---

## Repository Structure

Grid_Based_Planning_Tutorial/
  windows_demo/        # Standalone Python demo (no ROS required)
  ros2/                # ROS 2 Humble grid-planning demos
    src/
      gbp_common/          # Gazebo + map_server + RViz setup
      gbp_planning_demos/  # DFS/BFS/Dijkstra/A* on /map → nav_msgs/Path
  docs/                # Lecture slides and documents (PDF / PPT)
  README.md

---

## windows_demo

Standalone Python code intended to run on Windows (or any OS with Python 3).

(+ Numpy, Matplotlib, Pillow will be needed.)
