# Space-Time A*（时空 A*）路径规划

## 算法简介

Space-Time A\* 是标准 A\* 在**时间维度**上的扩展，将时间作为搜索空间的第三个维度，  
使算法能够在存在**动态障碍物**（障碍物位置随时间变化）的环境中规划路径。

核心区别：

| 对比项 | 标准 A\* | Space-Time A\* |
|--------|---------|---------------|
| 节点表示 | $(x, y)$ | $(x, y, t)$，含时间步 |
| 代价 $g(n)$ | 累计路径长度 | **到达该节点所用的时间步数** |
| 障碍物类型 | 静态 | **动态**（每个时间步位置不同） |
| 适用场景 | 静态地图 | 多机器人协作、动态障碍环境 |

---

## 算法原理

### 状态空间

每个节点为 $(x, y, t)$：
- $x, y$：在栅格地图中的位置
- $t$：当前时间步（整数）

### 代价函数

$$f(n) = g(n) + h(n)$$

- $g(n) = t$（到达节点 $n$ 所用的时间步数，而非路径长度）
- $h(n)$：曼哈顿距离或欧氏距离（忽略时间维度）

这样设计使路径**时间最优**：最少时间步到达目标。

### 动态障碍处理

在每个时间步 $t$，障碍物占据不同的栅格位置。  
节点 $(x, y, t)$ 被视为不可达，当且仅当在时间步 $t$ 时 $(x, y)$ 被障碍物占据。

```
时间步 t=0：障碍物在 A 位置  → 搜索时跳过 (A, 0)
时间步 t=1：障碍物在 B 位置  → 搜索时跳过 (B, 1)
时间步 t=2：障碍物在 C 位置  → 搜索时跳过 (C, 2)
```

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `SpaceTimeAStar.py` | Space-Time A\* 主算法 |
| `GridWithDynamicObstacles.py` | 含动态障碍物的栅格地图模型 |
| `Node.py` | 节点数据结构（含时间步） |
| `BaseClasses.py` | 规划器基类接口定义 |
| `Plotting.py` | 路径可视化工具 |

---

## 快速使用

### 安装依赖

```bash
pip install numpy matplotlib
```

### 直接运行演示

```bash
cd PathPlanning/SpaceTimeAStar
python SpaceTimeAStar.py
```

### 在代码中调用

```python
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))

import matplotlib
matplotlib.use('Agg')

from GridWithDynamicObstacles import Grid, ObstacleArrangement, Position
from SpaceTimeAStar import SpaceTimeAStar

# 创建含动态障碍物的栅格地图
#   width, height : 地图尺寸（格子数）
#   obstacle_arrangement : 障碍物排列方式（枚举值）
grid = Grid(
    width=10,
    height=10,
    obstacle_arrangement=ObstacleArrangement.ARRANGEMENT_1
)

# 定义起点和终点
start = Position(x=0, y=0)
goal  = Position(x=9, y=9)

# 执行规划，返回 NodePath（含路径节点列表和时间步序列）
path = SpaceTimeAStar.plan(grid=grid, start=start, goal=goal, verbose=True)

if path:
    print(f"找到路径，共 {len(path.nodes)} 个时间步")
    for node in path.nodes:
        print(f"  t={node.time}: ({node.position.x}, {node.position.y})")
```

---

## ObstacleArrangement 说明

`GridWithDynamicObstacles.py` 中预设了若干障碍物运动场景：

| 枚举值 | 场景描述 |
|--------|---------|
| `ARRANGEMENT_1` | 单个障碍物水平穿越地图 |
| `ARRANGEMENT_2` | 多个障碍物交叉运动 |

可自定义障碍物轨迹：

```python
from GridWithDynamicObstacles import Grid, Position

# 自定义：每个时间步指定障碍物位置列表
custom_obstacles = {
    0: [Position(3, 5)],
    1: [Position(4, 5)],
    2: [Position(5, 5)],
    3: [Position(6, 5)],
}
grid = Grid(width=10, height=10, dynamic_obstacles=custom_obstacles)
```

---

## 算法复杂度与特性

| 指标 | 说明 |
|------|------|
| 状态空间 | 三维 $(x, y, t)$，状态数 = 地图大小 × 最大时间步 |
| 时间复杂度 | $O(W \times H \times T_{max} \times \log(W \times H \times T_{max}))$ |
| 最优性 | ✅ 时间最优（$h$ 可接受时） |
| 适用场景 | 多机器人系统、动态环境导航、仓储 AGV 调度 |

---

## 与 UR5 的关系

> Space-Time A\* 适用于需要处理**动态障碍物**（如其他机器人、移动人员）的场景。  
> 在 UR5 多臂协作或人机协作场景中，可用于规划考虑其他机器人/人员运动轨迹的无碰撞路径。  
> 单臂静态环境中建议直接使用 **RRT\*** 或标准 **A\***。

---

## 参考文献

- Silver, D. (2005). *Cooperative Pathfinding*. AIIDE 2005.
- [Space-Time A\* 原始参考](https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf)
