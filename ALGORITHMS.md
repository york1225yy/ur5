# 机械臂路径规划算法学习指南

> 面向 UR5 机械臂项目的路径规划算法系统学习文档。  
> 全部学习代码基于 **Python**，无需额外编译环境。

---

## 一、核心概念速览

路径规划（Path Planning）解决的核心问题：

```
给定起始位姿 q_start 和目标位姿 q_goal，
在满足运动学约束和无碰撞约束的前提下，
求一条从 q_start 到 q_goal 的可行轨迹。
```

机械臂路径规划分为两类空间：

| 规划空间 | 含义 | 适用算法 |
|---------|------|---------|
| **关节空间（C-Space）** | 以各关节角度为维度的配置空间，UR5 为 6 维 | RRT、RRT*、PRM |
| **笛卡尔空间（Task Space）** | 末端执行器的 XYZ + 姿态空间 | A*（离散栅格）、人工势场法 |

---

## 二、主要算法原理与对比

### 2.1 A*（A-Star）

**核心思想**：启发式搜索，在栅格地图上寻找代价最小路径。

**评估函数**：

$$f(n) = g(n) + h(n)$$

- $g(n)$：从起点到节点 $n$ 的实际代价
- $h(n)$：从节点 $n$ 到终点的启发式估计代价（常用欧氏距离或曼哈顿距离）
- 当 $h(n) = 0$ 退化为 Dijkstra；当 $h(n)$ 完美时最优最快

**特点**：

| 优点 | 缺点 |
|------|------|
| 保证找到最优路径（若 $h$ 可接受） | 高维空间（如6自由度关节空间）内存爆炸 |
| 在低维栅格地图中效率高 | 不适合连续高维C空间直接使用 |
| 实现简单，易于理解 | 需预先构建栅格地图 |

**在 UR5 中的应用**：适合在 2D/3D 工作空间的离散栅格中规划末端轨迹，不直接用于6维关节空间。

---

### 2.2 RRT（Rapidly-exploring Random Tree，快速随机扩展树）

**核心思想**：通过随机采样在连续高维 C-Space 中增量式构建搜索树，天然适合高维机械臂关节空间。

**算法步骤**：

```
1. 初始化树 T = {q_start}
2. 重复以下步骤直到找到目标：
   a. 随机采样配置点 q_rand（以一定概率直接采样 q_goal）
   b. 在树 T 中找最近节点 q_near
   c. 从 q_near 向 q_rand 方向步进 Δ，得到 q_new
   d. 碰撞检测：若路段 q_near → q_new 无碰撞，则加入树
3. 当 q_new 足够接近 q_goal 时，回溯路径
```

**特点**：

| 优点 | 缺点 |
|------|------|
| 天然支持高维空间（适合6DOF机械臂） | **概率完备**但非最优（路径锯齿多） |
| 无需预建地图，适合复杂障碍 | 路径质量随机性大，重复性差 |
| 实现相对简单 | 在狭窄通道中效率低 |

**在 UR5 中的应用**：MoveIt2 内部默认使用 RRT 系列规划器（OMPL）进行关节空间规划。

---

### 2.3 RRT*（RRT-Star）

**在 RRT 基础上增加两步优化**，使路径渐进最优：

```
额外步骤1 - Choose Parent（选择最优父节点）：
  在 q_new 的邻域内，选择到 q_new 代价最小的节点作为父节点

额外步骤2 - Rewire（重连优化）：
  检查 q_new 是否能降低邻域内其他节点的代价，若可以则重连
```

**代价对比**：

$$\text{cost}^*(q_{new}) = \min_{q \in \text{Near}(q_{new})} \left[ \text{cost}(q) + d(q, q_{new}) \right]$$

**特点**：

| 对比 RRT | RRT* |
|---------|------|
| 路径不最优 | 随采样点增多**渐进最优** |
| 速度略快 | 速度略慢（多了邻域搜索） |
| 适合快速验证可行性 | 适合需要高质量路径的场景 |

---

### 2.4 其他算法简介

| 算法 | 适用场景 | Python 复杂度 |
|------|---------|-------------|
| **Dijkstra** | 无权重/均匀代价栅格地图，A* 特例 | 低 |
| **D* Lite** | 动态环境（障碍物实时变化），增量式重规划 | 中 |
| **PRM（概率路图法）** | 多查询场景，预建路图，适合静态环境 | 中 |
| **人工势场法（APF）** | 实时避障，计算量小，但有局部极小值问题 | 低 |
| **CHOMP/STOMP** | 梯度优化，对已有路径进行光滑化 | 高 |
| **Bi-RRT（双向RRT）** | 从起点和终点同时扩展，效率更高 | 中 |

---

## 三、GitHub 开源学习项目汇总

### 3.1 综合框架（强烈推荐先学）

| 仓库 | Stars | 包含算法 | 说明 | 链接 |
|------|-------|---------|------|------|
| **AtsushiSakai/PythonRobotics** | ⭐29613 | A\*、Dijkstra、RRT、RRT\*、PRM、D\*、APF 等 30+ 算法 | **机器人算法圣经**，每个算法独立 Python 文件+注释+动画演示，**首选** | [GitHub](https://github.com/AtsushiSakai/PythonRobotics) |

**PythonRobotics 中与 UR5 最相关的路径规划模块**：

```
PathPlanning/
├── AStar/                    # A* 栅格路径规划
├── RRTStar/                  # RRT* 高维路径规划
├── RRT/                      # 基础 RRT
├── BidirectionalAStar/       # 双向 A*，效率提升2x
├── ProbabilisticRoadMap/     # PRM，适合多次查询
└── DStarLite/                # D* Lite，动态障碍环境
```

---

### 3.2 A* 专项学习

| 仓库 | Stars | 特点 | 链接 |
|------|-------|------|------|
| arunumd/A_Star_Algorithm_Path_Planning | ⭐17 | 2D 静态障碍地图，可视化清晰，代码简洁 | [GitHub](https://github.com/arunumd/A_Star_Algorithm_Path_Planning) |
| jacobsayono/a-star-algorithm | ⭐8 | 欧氏距离启发函数，注释详细，适合入门 | [GitHub](https://github.com/jacobsayono/a-star-algorithm) |
| Pradeep-Gopal/Astar_Turtlebot_ROS_Gazebo | ⭐8 | A\* + ROS + Gazebo 仿真，可参考 ROS2 集成思路 | [GitHub](https://github.com/Pradeep-Gopal/Astar_Turtlebot_ROS_Gazebo) |

---

### 3.3 RRT / RRT* 专项学习

| 仓库 | Stars | 特点 | 链接 |
|------|-------|------|------|
| Abeilles14/Velocity-Obstacle-and-Motion-Planning | ⭐66 | RRT\* + 速度障碍法，双臂抓放任务，含动画演示 | [GitHub](https://github.com/Abeilles14/Velocity-Obstacle-and-Motion-Planning) |
| srnand/Mobile-Robots-Autonomous-Navigation | ⭐39 | RRT、RRT\*（Dubin曲线变种）+ Gazebo 仿真 | [GitHub](https://github.com/srnand/Mobile-Robots-Autonomous-Navigation) |
| oskarnatan/RRT-Path-Planning | ⭐9 | 室内多障碍物 RRT，纯 Python，结构清晰 | [GitHub](https://github.com/oskarnatan/RRT-Path-Planning) |

---

### 3.4 ROS2 集成路径规划

| 仓库 | Stars | 特点 | 链接 |
|------|-------|------|------|
| Dong-Chengteng/ros2-robot-navigation-exploration | ⭐11 | 双向A\* + DWA局部规划 + RRT探索，完整 ROS2 包 | [GitHub](https://github.com/Dong-Chengteng/ros2-robot-navigation-exploration) |

---

### 3.5 D* Lite（动态环境）

| 仓库 | Stars | 特点 | 链接 |
|------|-------|------|------|
| EricChen0104/D_star_lite_Algorithm_PYTHON | ⭐2 | D\* Lite 2D 实时可视化，障碍物动态变化演示 | [GitHub](https://github.com/EricChen0104/D_star_lite_Algorithm_PYTHON) |

---

## 四、算法选型指南（面向 UR5 项目）

```
需求分析
│
├── 静态障碍物 + 高维关节空间（主要场景）
│   ├── 首选：RRT* （MoveIt2 内置 OMPL，直接可用）
│   └── 备选：PRM  （预建路图，多次查询效率高）
│
├── 2D/3D 工作空间末端轨迹规划
│   ├── 首选：A*   （栅格地图，最优路径保证）
│   └── 备选：双向A* （搜索效率提升约2倍）
│
├── 动态障碍物实时变化
│   └── 首选：D* Lite （增量式重规划，避免全局重算）
│
└── 路径平滑优化（在规划路径基础上）
    └── 后处理：CHOMP（梯度优化）或 B-Spline 插值
```

---

## 五、UR5 + MoveIt2 规划器说明

MoveIt2 通过 **OMPL（Open Motion Planning Library）** 提供规划算法，默认使用 RRT-Connect：

```bash
# 查看 MoveIt2 可用规划器
ros2 param get /move_group ompl/planning_plugins

# 常用规划器配置（在 ompl_planning.yaml 中设置）
# RRTConnect  - 默认，双向RRT，速度快
# RRTstar     - 渐进最优，适合需要高质量路径场景
# PRMstar     - 多查询场景
# LBKPIECE1   - 适合高维空间
```

**学习路径（与代码结合）**：

```
Step 1：运行 PythonRobotics/PathPlanning/AStar/a_star.py
        → 理解代价函数 f(n)=g(n)+h(n) 的搜索过程

Step 2：运行 PythonRobotics/PathPlanning/RRT/rrt.py
        → 观察树的随机扩展动画，理解采样策略

Step 3：运行 PythonRobotics/PathPlanning/RRTStar/rrt_star.py
        → 对比 RRT vs RRT* 的路径质量差异

Step 4：在 ROS2 + Gazebo 中运行 MoveIt2
        → 观察 OMPL 如何在6维关节空间执行上述算法
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5

Step 5：阅读 Dong-Chengteng/ros2-robot-navigation-exploration
        → 理解如何在 ROS2 中自定义集成路径规划算法
```

---

## 六、快速上手（PythonRobotics）

```bash
# 克隆仓库
git clone https://github.com/AtsushiSakai/PythonRobotics.git
cd PythonRobotics

# 安装依赖
pip install -r requirements.txt   # numpy, matplotlib, scipy 等

# 运行 A* 示例（含实时动画）
python PathPlanning/AStar/a_star.py

# 运行 RRT 示例
python PathPlanning/RRT/rrt.py

# 运行 RRT* 示例
python PathPlanning/RRTStar/rrt_star.py

# 运行双向 A*
python PathPlanning/BidirectionalAStar/bidirectional_a_star.py
```

---

## 七、参考文献

- LaValle, S. M. (1998). *Rapidly-exploring random trees: A new tool for path planning*. TR 98-11, CS Dept., Iowa State Univ.
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE TSSC.
- Karaman, S., & Frazzoli, E. (2011). *Sampling-based Algorithms for Optimal Motion Planning*. IJRR.
- [OMPL 官方文档](https://ompl.kavrakilab.org/)
- [MoveIt2 Planning Pipeline](https://moveit.picknik.ai/main/doc/concepts/planning_pipeline/planning_pipeline.html)
