# RRT（快速随机扩展树）路径规划

## 算法简介

RRT（Rapidly-exploring Random Tree）是一种**基于随机采样的运动规划算法**，通过在高维配置空间（C-Space）中增量式构建搜索树，天然适合 UR5 等多自由度机械臂的**关节空间规划**。

### 算法流程

```
初始化：T = {q_start}

循环直到找到路径：
  1. q_rand ← 随机采样（以概率 p 直接采样 q_goal）
  2. q_near ← T 中距 q_rand 最近的节点
  3. q_new  ← 从 q_near 向 q_rand 方向步进 expand_dis
  4. 碰撞检测：若 q_near → q_new 无碰撞，则加入 T
  5. 若 q_new 到 q_goal 距离 < 阈值，路径找到

回溯：从 q_new 沿父节点链回溯到 q_start
```

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `rrt.py` | 标准 RRT，是 RRT\* 系列的基类 |
| `rrt_with_pathsmoothing.py` | RRT + 路径平滑后处理（去除多余中间节点） |

---

## 快速使用

### 直接运行演示

```bash
python rrt.py                     # 标准 RRT
python rrt_with_pathsmoothing.py  # RRT + 路径平滑
```

### 在代码中调用（标准 RRT）

```python
from rrt import RRT

# 障碍物列表：每个元素 [x, y, radius]
obstacle_list = [
    (5,  5,  1),   # 圆形障碍物，圆心(5,5)，半径1
    (3,  6,  2),
    (3,  8,  2),
    (3, 10,  2),
    (7,  5,  2),
    (9,  5,  2),
]

rrt = RRT(
    start=[0, 0],           # 起点 [x, y]
    goal=[6, 10],           # 终点 [x, y]
    rand_area=[-2, 15],     # 随机采样范围 [min, max]
    obstacle_list=obstacle_list,
    expand_dis=1.0,         # 每步扩展距离
    path_resolution=0.5,    # 路径插值分辨率
    goal_sample_rate=5,     # 直接采样目标点的概率（%）
    max_iter=500,           # 最大迭代次数
)

path = rrt.planning(animation=False)  # path 为 [[x,y], ...] 列表，若失败返回 None

if path:
    print(f"找到路径，共 {len(path)} 个路径点")
```

### 在代码中调用（RRT + 路径平滑）

```python
from rrt_with_pathsmoothing import RRTWithPathSmoothing

rrt_smooth = RRTWithPathSmoothing(
    start=[0, 0],
    goal=[6, 10],
    rand_area=[-2, 15],
    obstacle_list=obstacle_list,
)

path = rrt_smooth.planning(animation=False)
```

---

## 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `expand_dis` | 每步扩展距离，影响树的粒度 | 障碍物间距的 1/3 ~ 1/2 |
| `path_resolution` | 碰撞检测插值步长，越小越精确但越慢 | `expand_dis / 2` |
| `goal_sample_rate` | 直接采样目标点概率（%），影响收敛速度 | 5 ~ 20 |
| `max_iter` | 最大迭代次数，超出则返回 None | 500 ~ 2000 |
| `robot_radius` | 机器人安全半径（m） | 实际半径 × 1.1 |

---

## 算法特性

| 特性 | 值 |
|------|-----|
| 最优性 | ❌ 不保证（路径有锯齿） |
| 完备性 | ✅ 概率完备（迭代足够多必收敛） |
| 高维适用性 | ✅ 天然支持（不受维度诅咒影响） |
| 动态障碍 | ❌ 不支持（需重规划） |

---

## 路径平滑原理

`rrt_with_pathsmoothing.py` 在 RRT 找到路径后进行后处理：

```
原始路径：P0 → P1 → P2 → P3 → P4 → P5
平滑处理：检查 P0 → P2 是否无碰撞，若是则删除 P1
结果路径：P0 ──────→ P2 → P3 → P4 → P5
```

反复迭代直到无法继续合并，路径节点数显著减少。

---

## 适用场景

- UR5 关节空间的可行路径快速验证
- PyBullet / Gazebo 仿真环境中的运动规划
- 作为 MoveIt2 OMPL 规划器的理解基础
- 需要二次开发（自定义碰撞检测、代价函数）时
