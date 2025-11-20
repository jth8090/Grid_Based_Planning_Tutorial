### Grid Based Planning Tutorial

This repository contains all source code and lecture materials used in my tutorial series on **Grid-Based Planning**.

The goal of this project is to bridge three layers:

1. **Theory** – grids, occupancy grids, costmaps, inflation, classical search algorithms
2. **Standalone algorithms** – pure Python implementations with step-by-step visualization on Windows
3. **ROS 2 integration** – applying the same ideas to real `OccupancyGrid` maps in Gazebo + RViz

Everything here is organized so that you can:

* watch the lecture videos,
* open the slides,
* run the same code on Windows or on Ubuntu/ROS 2,
* and clearly see how the theory is used in practice.

---

## Repository Structure

```text
Grid_Based_Planning_Tutorial/
  windows_demo/                # Standalone Python demo (no ROS required)
  ros2/                        # ROS 2 Humble grid-planning demos
    src/
      gbp_common/              # Gazebo + map_server + RViz setup
      gbp_planning_demos/      # DFS/BFS/Dijkstra/A* on /map → nav_msgs/Path
  lecture_docs/                # Lecture slides and documents (PDF / PPT)
  README.md
```

### `windows_demo/`

Standalone Python code intended to run on Windows (or any OS with Python 3).

Main entry point:

```run_compare.py``` 

<img width="1422" height="892" alt="run_compare" src="https://github.com/user-attachments/assets/15fea1cc-de63-40ec-a864-8bbb67b7748a" />

→ opens an interactive window where you can edit a grid and compare search algorithms.

Supporting modules:

* `algorithms/`

  * `bfs.py`
  * `dfs.py`
  * `dijkstra.py`
  * `astar.py`
* `utils/`

  * `gridmap.py`
  * `costmap.py`
  * `search_utils.py`
  * (plus other helpers if needed)

**Key Features**

* Edit a 2D grid:

  * set start/goal cells,
  * draw obstacles,
  * adjust costs.
  
* Visualize and compare:

  * **DFS** (Depth-First Search)
  * **BFS** (Breadth-First Search)
  * **Dijkstra’s Algorithm**
  * **A*** (A-star)
  
* Step-by-step animation of:

  * **frontier** vs **visited** cells,
  * the final path,
  * how each algorithm expands nodes differently.
  * 
* Behavior and notation are aligned with the lecture slides (Lectures 2–4).

> Note: required Python packages are minimal (numpy, matplotlib, pillow).

---

### `ros2/src/gbp_common`

A minimal ROS 2 (Humble) package that provides a **common simulation and visualization environment**:

* a simple **Gazebo** world,
* a **map_server** that publishes a static `nav_msgs/OccupancyGrid` on `/map`,
* an **RViz** configuration that shows:

  * the map,
  * frames (e.g., `map`, `odom`, `base_link`),
  * and optionally a costmap view.

This package is mainly used to support **Lecture 1** topics in a real ROS 2 setup:

* world ↔ grid mapping (cell-center convention),
* row/column index conventions (`i` as vertical, `j` as horizontal),
* occupancy grids vs. costmaps,
* obstacle inflation.

**Basic usage (Ubuntu, ROS 2 Humble)**

```bash
cd ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gbp_common view_map.launch.py
```

Once launched, you can:

* open RViz,
* subscribe to `/map`,
* and visually confirm that the grid index/coordinate conventions in the slides match the real simulation.

---

### `ros2/src/gbp_planning_demos`

ROS 2 demos that take a real occupancy grid from `/map` and run the same algorithms (DFS, BFS, Dijkstra, A*) used in `windows_demo`, but inside a ROS 2 workflow.

**Core node**: `simple_grid_planner` (Python)

* subscribes to `/map` (`nav_msgs/OccupancyGrid`),

* converts it to an internal `GridMap` representation (same cell-center convention as in the slides),

* selects the algorithm via ROS parameters:

  ```yaml
  algorithm: "dfs" | "bfs" | "dijkstra" | "astar"
  heuristic: "manhattan" | "octile" | "euclidean"  # for A*
  use_diagonal: true/false
  ```

* runs the planner,

* reconstructs the path from start to goal,

* publishes the result as `nav_msgs/Path` on `demo_path`,

* uses a **transient local** QoS profile for the path publisher so that RViz can still receive the last path even if the display is added later.

**Launch files**

The package typically provides the following launch files:

* `demo_dfs_bfs.launch.py`
  Runs two planners:

  * `planner_dfs` → publishes `planner_dfs/demo_path`
  * `planner_bfs` → publishes `planner_bfs/demo_path`

* `demo_dijkstra.launch.py`
  Compares **BFS vs. Dijkstra** on the same map and start/goal.

* `demo_astar.launch.py`
  Compares **Dijkstra vs. A*** (with selectable heuristic).

* `demo_all_algorithms.launch.py`
  Runs **DFS, BFS, Dijkstra, A*** all at once:

  * `planner_dfs/demo_path`
  * `planner_bfs/demo_path`
  * `planner_dijkstra/demo_path`
  * `planner_astar/demo_path`

**Example: run all algorithms**

```bash
cd ros2
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1 – Gazebo + map + RViz
ros2 launch gbp_common view_map.launch.py

# Terminal 2 – planners
ros2 launch gbp_planning_demos demo_all_algorithms.launch.py
```

In RViz:

1. Add four `Path` displays.
2. Set their topics to:

   * `planner_dfs/demo_path`
   * `planner_bfs/demo_path`
   * `planner_dijkstra/demo_path`
   * `planner_astar/demo_path`
3. Give each path a different color.
4. For each `Path` display, set QoS:

   * Reliability: **Reliable**
   * Durability: **Transient Local**

This lets you visually compare:

* search patterns (how each algorithm explores the grid),
* the resulting path shapes,
* the effect of using a heuristic in A* vs. pure Dijkstra.

---

## Lecture Materials

All slides used in the videos are stored in the `lecture_docs/` directory:

* `lecture_docs/GBP_Lecture_1.pdf` – Introduction, grids, occupancy grids, costmaps, inflation
* `lecture_docs/GBP_Lecture_2.pdf` – DFS & BFS (search order, complexity, limitations)
* `lecture_docs/GBP_Lecture_3.pdf` – Dijkstra’s algorithm (graph interpretation, optimality proof, relation to BFS)
* `lecture_docs/GBP_Lecture_4.pdf` – A* search (heuristics, admissibility, consistency, optimality proof)

You can follow the slides while:

* running `windows_demo` on a local grid, or
* running the ROS 2 demos on an actual occupancy map.

---

## Lecture Videos (YouTube)

The full tutorial series is available on YouTube (in Korean).

* **Tutorial Lecture(Youtube)**:
  👉 [Grid Based Planning Tutorial – YouTube]([https://www.youtube.com/XXXXXXXXXXXX](https://youtube.com/playlist?list=PLfHEg65-de01IgZCHuQG21f6_7UmgTTKS&si=DnkbHJqKHTHSfzqB))

Each lecture roughly follows the same pattern:

1. Theory on the slides (from `lecture_docs/`),
2. Walkthrough of the Python implementation,
3. ROS 2 demo on the same concept.
4. 
---

## Getting Started (Quick Overview)

### Windows standalone demo

1. Install Python 3.x.

2. Install any required packages (e.g., `numpy`, `matplotlib`) according to comments or a `requirements.txt` file.

3. Run:

   ```bash
   python windows_demo/run_compare.py
   ```

4. Edit the grid and press the corresponding buttons to run DFS, BFS, Dijkstra, or A*.
   Watch how the frontier and visited sets grow, and compare the final paths.

### ROS 2 demos (Ubuntu, Humble)

1. Clone this repository into a workspace (e.g., `~/gbp_ws/ros2`).

2. Build:

   ```bash
   cd ros2
   colcon build
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

3. Launch the common environment:

   ```bash
   ros2 launch gbp_common view_map.launch.py
   ```

   ```bash
   # It is important that activation of map_server lifecycle
   # Type below in another terminal.
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activation
   ```

4. In another terminal, launch one of the demos, for example:

   ```bash
   ros2 launch gbp_planning_demos path_planning.launch.py
   ```

5. Open RViz and add `Path` displays for each planner topic as described above.


---

## Contact

* Author: **Taehyun Jeong (정태현)**
* E-mail: jth8090@khu.ac.kr
* GitHub: [@jth8090](https://github.com/jth8090)
