# RRT*（RRT-Star）渐进最优路径规划

## 算法简介

RRT\* 在 RRT 基础上增加两步关键操作，使路径质量**随采样点增多而渐进收敛到最优**。

### 相比 RRT 增加的两步

```
RRT 流程 + 以下两步：

步骤1 - Choose Parent（选择最优父节点）
  在 q_new 邻域 Near(q_new, r) 内，找到使 cost(q_new) 最小的父节点：
  cost*(q_new) = min_{q ∈ Near} [ cost(q) + dist(q, q_new) ]

步骤2 - Rewire（重连优化）
  检查 q_new 是否能降低邻域内其他节点的代价：
  for q ∈ Near(q_new, r):
      if cost(q_new) + dist(q_new, q) < cost(q):
          将 q 的父节点更新为 q_new
```

### 代价收敛过程

随采样点增加，树中路径代价单调递减，最终收敛到最优解。

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `rrt_star.py` | RRT\* 完整实现，继承自 `RRT/rrt.py` 中的 `RRT` 基类 |

> **注意**：`rrt_star.py` 依赖 `../RRT/rrt.py`，两个文件需放在同级目录下。

---

## 快速使用

### 直接运行演示

```bash
# 需要在 PathPlanning/ 根目录执行，以保证导入路径正确
cd ..
python -m RRTStar.rrt_star
```

### 在代码中调用

```python
import sys
sys.path.append('..')       # 指向 PathPlanning/ 目录

from RRTStar.rrt_star import RRTStar

obstacle_list = [
    (5, 5, 1),
    (3, 6, 2),
    (3, 8, 2),
    (3, 10, 2),
    (7, 5, 2),
    (9, 5, 2),
]

rrt_star = RRTStar(
    start=[0, 0],
    goal=[6, 10],
    rand_area=[-2, 15],
    obstacle_list=obstacle_list,
    expand_dis=1.0,
    path_resolution=0.5,
    goal_sample_rate=20,
    max_iter=300,
    connect_circle_dist=50.0,   # 邻域搜索半径，影响 rewire 范围
    search_until_max_iter=False, # True: 跑满 max_iter 再返回最优路径
    robot_radius=0.0,
)

path = rrt_star.planning(animation=False)

if path:
    print(f"路径代价: {rrt_star.path_cost}")
    print(f"路径点数: {len(path)}")
```

---

## 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `connect_circle_dist` | rewire 邻域搜索半径，越大路径越优但越慢 | 30 ~ 100 |
| `search_until_max_iter` | `False`：找到路径即返回；`True`：跑满迭代持续优化 | 演示用 True，集成用 False |
| `max_iter` | 最大迭代次数 | 300 ~ 1000 |

---

## RRT vs RRT\* 对比

| 对比项 | RRT | RRT\* |
|--------|-----|-------|
| 最优性 | ❌ 不保证 | ✅ 渐进最优 |
| 每次迭代耗时 | 低 | 略高（邻域搜索） |
| 路径质量 | 锯齿多，较长 | **随迭代持续改善** |
| 收敛速度 | 快速找到可行路径 | 找到可行路径后继续优化 |
| 内存占用 | 低 | 略高 |

**实践建议**：用 RRT 快速验证路径可行性，用 RRT\* 生成用于实际执行的高质量路径。

---

## 在 MoveIt2 中的对应

MoveIt2 通过 OMPL 库内置 RRT\* 规划器，直接可用：

```yaml
# ur_moveit_config/config/ompl_planning.yaml
ur5:
  default_planner_config: RRTstar
  planner_configs:
    - RRTstar
    - RRTConnect   # 默认，速度快
    - PRMstar
```

```bash
# 运行时指定规划器
ros2 run moveit2_tutorials move_group_interface_tutorial \
  --ros-args -p planning_plugin:=ompl_interface/OMPLPlanning
```

---

## 适用场景

- UR5 关节空间高质量路径规划
- 需要路径代价最优的抓取任务
- MoveIt2/OMPL 核心算法的学习基础
