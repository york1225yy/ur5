# Informed RRT*（椭圆加速 RRT*）路径规划

## 算法简介

Informed RRT\* 在 RRT\* 基础上增加**自适应采样区域缩减**：一旦找到初始可行路径，后续采样**仅在以起终点为焦点的椭圆区域内进行**，显著加速收敛。

### 核心思想

设初始路径代价为 $c_{best}$，任何更优的路径必须满足：

$$\text{dist}(q_{start}, q) + \text{dist}(q, q_{goal}) < c_{best}$$

满足此条件的点集恰好是一个以 $q_{start}$、$q_{goal}$ 为焦点的**椭圆**，随着路径优化，椭圆不断收缩，采样效率大幅提升。

```
初始阶段（RRT* 模式）：
  ┌──────────────────────────────┐
  │    全空间均匀采样              │
  │  Start ──────────────► Goal   │
  └──────────────────────────────┘

找到初解后（Informed 模式）：
       ╭──────────────╮
      ╭╯ 椭圆采样区域 ╰╮
     Start ──────► Goal
      ╰╮              ╭╯
       ╰──────────────╯
```

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `informed_rrt_star.py` | Informed RRT\* 完整实现，含椭圆采样可视化 |

---

## 快速使用

### 直接运行演示

```bash
# 在 PathPlanning/ 根目录执行
python -m InformedRRTStar.informed_rrt_star
```

动画中可以看到蓝色椭圆随路径优化不断收缩。

### 在代码中调用

```python
import sys
sys.path.append('.')       # 指向 PathPlanning/ 目录

from InformedRRTStar.informed_rrt_star import InformedRRTStar

obstacle_list = [
    (5,  5, 0.5),
    (9,  6, 1),
    (7,  5, 1),
    (1,  5, 1),
    (3,  5, 1),
    (7,  3, 1),
    (6, 13, 0.5),
    (9, 11, 1),
    (8, 11, 1),
]

planner = InformedRRTStar(
    start=[0, 0],
    goal=[6, 10],
    obstacle_list=obstacle_list,
    rand_area=[-2, 18],
    expand_dis=0.5,
    path_resolution=0.1,
    goal_sample_rate=10,
    max_iter=200,
    connect_circle_dist=50.0,
    search_until_max_iter=True,   # 跑满 max_iter 以充分优化
)

path = planner.planning(animation=False)

if path:
    print(f"最终路径代价: {planner.best_path_length:.2f}")
```

---

## 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `search_until_max_iter` | 建议设为 `True`，使椭圆充分收缩 | `True` |
| `max_iter` | 迭代次数越多椭圆越小、路径越优 | 200 ~ 1000 |
| `connect_circle_dist` | rewire 邻域半径 | 30 ~ 100 |

---

## 三种算法收敛速度对比

| 算法 | 找到初解 | 后续优化速度 | 最终路径质量 |
|------|---------|------------|------------|
| RRT\* | 快 | 慢（全局采样） | 好（时间足够时） |
| **Informed RRT\*** | 快 | **快**（椭圆采样） | **更好**（同等时间内） |
| BIT\* | 较快 | 最快（批量+图搜索） | 最好 |

---

## 依赖说明

`informed_rrt_star.py` 依赖同目录下的 `../utils/angle.py`（2D 旋转矩阵）。  
已通过 `sys.path.append` 正确配置，无需手动修改。

---

## 适用场景

- 计算时间有限但需要高质量路径的场景
- 工作空间较大、障碍物较多的环境
- UR5 关节空间轨迹规划的算法研究
