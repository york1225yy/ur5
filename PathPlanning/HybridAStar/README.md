# Hybrid A*（混合 A*）路径规划

## 算法简介

Hybrid A\* 是对标准 A\* 的扩展，专为**非完整性约束（Non-holonomic）运动系统**设计，如汽车、差速机器人等。  
它在离散栅格搜索的基础上，融入了连续运动学模型，使输出路径**天然满足曲率约束**，可被车辆/非完整机器人直接执行。

> **与标准 A\* 的核心区别**：标准 A\* 输出的路径由格点折线组成，无法被非完整系统跟踪；  
> Hybrid A\* 的每段路径由 **Reeds-Shepp 曲线**或圆弧组成，满足最大曲率限制。

---

## 算法原理

### 状态空间

标准 A\* 的节点是 $(x, y)$，Hybrid A\* 的节点是 $(x, y, \theta)$：

| 维度 | 含义 |
|------|------|
| $x, y$ | 在栅格地图中的位置 |
| $\theta$ | 车辆/机器人的朝向角 |

### 代价函数

$$f(n) = g(n) + h(n)$$

- $g(n)$：实际行驶代价，含**倒车惩罚**、**方向切换惩罚**、**转向角惩罚**
- $h(n)$：启发式代价，取以下两者的最大值：
  - **非完整性启发**：Reeds-Shepp 路径长度（考虑转向约束）
  - **完整性启发**：动态规划障碍物代价（考虑障碍物分布）

### Reeds-Shepp 曲线

Reeds-Shepp 路径是两点间满足最大曲率约束的**最短路径**，由若干圆弧和直线段组成，  
每段可以前进（+）或倒退（-），共 5 种基本曲线类型，48 种组合。

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `hybrid_a_star.py` | Hybrid A\* 主算法，含完整可视化 |
| `reeds_shepp_path_planning.py` | Reeds-Shepp 曲线规划器（依赖库） |
| `car.py` | 车辆运动学模型与碰撞检测（依赖库） |
| `dynamic_programming_heuristic.py` | 基于动态规划的障碍物代价启发函数（依赖库） |
| `utils/angle.py` | 角度归一化工具函数（依赖库） |

---

## 快速使用

### 安装依赖

```bash
pip install scipy numpy matplotlib
```

### 直接运行演示

```bash
cd PathPlanning/HybridAStar
python hybrid_a_star.py
```

运行后弹出动画窗口，显示车辆从起点沿满足转向约束的路径到达终点的过程。

### 在代码中调用

```python
import sys, pathlib
sys.path.append(str(pathlib.Path(__file__).parent))

import matplotlib
matplotlib.use('Agg')   # 无显示器时使用

from hybrid_a_star import hybrid_a_star, Config
import numpy as np

# 定义障碍物（矩形边界 + 中间障碍）
ox, oy = [], []
for i in range(60):           # 下边界
    ox.append(i); oy.append(0.0)
for i in range(60):           # 上边界
    ox.append(i); oy.append(60.0)
for i in range(61):           # 左边界
    ox.append(0.0); oy.append(i)
for i in range(61):           # 右边界
    ox.append(60.0); oy.append(i)
for i in range(40):           # 中间障碍墙
    ox.append(20.0); oy.append(i)

# 起点(x, y, yaw[rad]) 和终点
start = [10.0, 10.0, np.deg2rad(90.0)]
goal  = [50.0, 50.0, np.deg2rad(-90.0)]

# 执行规划
path = hybrid_a_star(start, goal, ox, oy, xy_resolution=2.0, yaw_resolution=np.deg2rad(15.0))

if path:
    print(f"找到路径，共 {len(path.x_list)} 个路径点")
    print(f"x: {path.x_list[:5]} ...")
```

---

## 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `xy_resolution` | 栅格分辨率（m），影响规划精度和速度 | 1.0 ~ 2.0 |
| `yaw_resolution` | 朝向角离散化精度（rad） | `np.deg2rad(10~20)` |
| `N_STEER` | 转向角采样数量 | 20 |
| `SB_COST` | 倒车切换惩罚系数 | 100.0 |
| `BACK_COST` | 倒车代价系数 | 5.0 |

---

## 算法复杂度与特性

| 指标 | 说明 |
|------|------|
| 状态空间 | 三维 $(x, y, \theta)$，比标准 A\* 高一维 |
| 最优性 | 近似最优（受栅格分辨率影响） |
| 完备性 | 概率完备 |
| 典型用途 | 自动泊车、AGV 路径规划、非完整移动机器人 |

---

## 与 UR5 的关系

> Hybrid A\* 主要面向**移动平台**（如 AGV、自动泊车），不直接用于 UR5 机械臂的关节空间规划。  
> 对于 UR5，推荐使用 **RRT\*** 或 **MoveIt2（OMPL）** 进行关节空间规划。  
> Hybrid A\* 可作为 UR5 **移动底座**（若有）的路径规划组件。

---

## 参考文献

- Dolgov, D., et al. (2008). *Practical Search Techniques in Path Planning for Autonomous Driving*. AAAI-08.
- Reeds, J. A., & Shepp, L. A. (1990). *Optimal paths for a car that goes both forwards and backwards*. Pacific Journal of Mathematics.
