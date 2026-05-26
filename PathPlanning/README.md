# 路径规划算法集合

本目录整理自 [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)，仅保留与 UR5 机械臂项目相关的 **A\* 族**和 **RRT 族**路径规划算法，全部基于 Python 实现。

---

## 目录结构

```
PathPlanning/
├── utils/                       # 公共工具函数
│   └── angle.py                 # 2D 旋转矩阵、角度归一化
│
├── AStar/                       # A* 及其变体（栅格搜索）
│   ├── a_star.py                # 标准 A*
│   ├── a_star_searching_from_two_side.py  # 双向 A*（Two-Side）
│   ├── a_star_variants.py       # A* 变体（对角线移动等）
│   └── README.md
│
├── BidirectionalAStar/          # 双向 A*（BidirectionalAStar 版）
│   ├── bidirectional_a_star.py
│   └── README.md
│
├── ThetaStar/                   # Theta*（任意角度 A*）
│   ├── theta_star.py
│   └── README.md
│
├── HybridAStar/                 # Hybrid A*（非完整系统，含 Reeds-Shepp）
│   ├── hybrid_a_star.py         # 主算法
│   ├── reeds_shepp_path_planning.py  # RS 曲线规划器
│   ├── car.py                   # 车辆运动学模型
│   ├── dynamic_programming_heuristic.py  # DP 启发函数
│   └── README.md
│
├── SpaceTimeAStar/              # Space-Time A*（动态障碍物）
│   ├── SpaceTimeAStar.py        # 主算法
│   ├── GridWithDynamicObstacles.py  # 动态障碍物地图模型
│   ├── Node.py                  # 节点数据结构
│   ├── BaseClasses.py           # 规划器基类
│   ├── Plotting.py              # 可视化工具
│   └── README.md
│
├── RRT/                         # 基础 RRT 及变体
│   ├── rrt.py                   # 标准 RRT
│   ├── rrt_with_pathsmoothing.py # RRT + 路径平滑后处理
│   ├── rrt_with_sobol_sampler.py # RRT-Sobol（Sobol 低差异采样）
│   ├── sobol/sobol.py            # Sobol 序列生成器
│   └── README.md
│
├── RRTStar/                     # RRT*（渐进最优）
│   ├── rrt_star.py
│   └── README.md
│
├── InformedRRTStar/             # Informed RRT*（椭圆采样加速）
│   ├── informed_rrt_star.py
│   └── README.md
│
└── BatchInformedRRTStar/        # BIT*（批量 Informed Trees）
    ├── batch_informed_rrt_star.py
    └── README.md
```

---

## 算法速查表

### A* 族（基于栅格/图搜索）

| 算法 | 文件 | 核心特点 | 适用场景 |
|------|------|---------|---------|
| **A\*** | `AStar/a_star.py` | 启发式搜索，$f=g+h$，保证最优 | 2D 工作空间末端轨迹 |
| **双向 A\*（Two-Side）** | `AStar/a_star_searching_from_two_side.py` | 从起点和终点同时扩展，速度约 2× | 较大地图，需要速度提升 |
| **双向 A\*（BidirectionalAStar）** | `BidirectionalAStar/bidirectional_a_star.py` | 双向同时扩展的另一实现版本 | 同上 |
| **Theta\*** | `ThetaStar/theta_star.py` | 任意角度移动，路径更自然平滑 | 需要连续平滑轨迹的场景 |
| **Hybrid A\*** | `HybridAStar/hybrid_a_star.py` | 融合连续运动学+Reeds-Shepp，满足曲率约束 | 非完整系统（AGV、移动底座） |
| **Space-Time A\*** | `SpaceTimeAStar/SpaceTimeAStar.py` | 时间作为第三维，处理动态障碍物 | 多机协作、人机协作场景 |

### RRT 族（基于随机采样）

| 算法 | 文件 | 核心特点 | 适用场景 |
|------|------|---------|---------|
| **RRT** | `RRT/rrt.py` | 随机扩展树，概率完备，不保证最优 | 高维 C-Space 快速验证可行性 |
| **RRT + 路径平滑** | `RRT/rrt_with_pathsmoothing.py` | RRT 基础上对锯齿路径做平滑后处理 | 对路径质量有一定要求时 |
| **RRT-Sobol** | `RRT/rrt_with_sobol_sampler.py` | Sobol 准随机序列采样，覆盖更均匀，收敛更快 | 需要均匀探索或可复现实验 |
| **RRT\*** | `RRTStar/rrt_star.py` | 加入 rewire 步骤，渐进最优 | 需要高质量路径（MoveIt2 默认使用） |
| **Informed RRT\*** | `InformedRRTStar/informed_rrt_star.py` | 找到初解后在椭圆区域内采样，收敛更快 | 需要快速收敛到最优路径 |
| **BIT\*** | `BatchInformedRRTStar/batch_informed_rrt_star.py` | 批量采样 + A\* 式搜索，效率最高 | 计算资源充足、追求最优效率 |

---

## 安装依赖

```bash
pip install numpy matplotlib scipy
```

---

## 算法选型建议（UR5 机械臂）

```
关节空间规划（6 DOF C-Space）
  └── RRT* / Informed RRT*        ← MoveIt2/OMPL 内置，直接可用

工作空间末端路径规划（2D/3D 栅格）
  └── 双向 A* / Theta*            ← 保证最优，平滑度更好

算法原型快速验证
  └── RRT                         ← 最简单，便于二次开发

追求渐进最优且收敛快
  └── Informed RRT* / BIT*        ← 找到初解后快速优化

非完整移动底座路径规划（AGV / 自动泊车）
  └── Hybrid A*                   ← 满足曲率约束，适合车辆类系统

多机器人协作 / 动态障碍物环境
  └── Space-Time A*               ← 时空三维搜索，时间最优
```

---

## 原始来源

代码整理自 [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)（MIT License）。
