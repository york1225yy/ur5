# 路径规划算法通俗指南

> 覆盖本项目 PathPlanning/ 下所有算法的原理、输入输出、路径存储与机械臂对接方法

---

## 目录

1. [算法输入输出总览](#一算法输入输出总览)
2. [路径存储格式与传递给机械臂](#二路径存储格式与传递给机械臂)
3. [A\* 系列算法通俗原理](#三a-系列算法通俗原理)
4. [RRT 系列算法通俗原理](#四rrt-系列算法通俗原理)
5. [算法横向对比表](#五算法横向对比表)
6. [UR5 实际对接指南](#六ur5-实际对接指南)

---

## 一、算法输入输出总览

### 1.1 A* 系列（网格类）

| 算法 | 文件 | 输入 | 输出格式 |
|------|------|------|----------|
| **A\*** | `AStar/a_star.py` | 障碍物坐标列表 `(ox, oy)`、网格分辨率、机器人半径、起点 `(sx,sy)`、终点 `(gx,gy)` | `rx, ry`：两个浮点列表（x 坐标序列、y 坐标序列），单位 m |
| **双向 A\*** | `BidirectionalAStar/bidirectional_a_star.py` | 同上 | `rx, ry`：同 A\* |
| **Theta\*** | `ThetaStar/theta_star.py` | 同上 | `rx, ry`：同 A\*，但路径更顺滑（直线段更多） |
| **双向 A\*（二分支）** | `AStar/a_star_searching_from_two_side.py` | 同上 | `rx, ry` |
| **A\* 变体集** | `AStar/a_star_variants.py` | 同上 + 可选启发函数类型 | `rx, ry` |
| **Hybrid A\*** | `HybridAStar/hybrid_a_star.py` | 起点/终点含朝向角 `[x, y, yaw]`、障碍物列表、xy 分辨率、yaw 分辨率 | `Path` 对象：`x_list, y_list, yaw_list, direction_list, cost` |
| **时空 A\*** | `SpaceTimeAStar/SpaceTimeAStar.py` | 网格地图 + 动态障碍物时间步信息 | 带时间戳的节点序列 `(x, y, t)` |

### 1.2 RRT 系列（采样类）

| 算法 | 文件 | 输入 | 输出格式 |
|------|------|------|----------|
| **RRT** | `RRT/rrt.py` | `start=[x,y]`、`goal=[x,y]`、`obstacle_list=[[x,y,r],…]`、`rand_area=[min,max]` | `path`：`[[x,y], [x,y], …]` 嵌套列表，从终点到起点排列 |
| **RRT + 路径平滑** | `RRT/rrt_with_pathsmoothing.py` | 同 RRT | `path`：平滑后的 `[[x,y], …]`，折点更少 |
| **RRT-Sobol** | `RRT/rrt_with_sobol_sampler.py` | 同 RRT | `path`：同 RRT 格式 |
| **RRT\*** | `RRTStar/rrt_star.py` | 同 RRT + `connect_circle_dist`（重连半径） | `path`：`[[x,y], …]`，路径质量更优 |
| **Informed RRT\*** | `InformedRRTStar/informed_rrt_star.py` | 同 RRT\* | `path`：`[[x,y], …]`，收敛更快 |
| **Batch Informed RRT\*** | `BatchInformedRRTStar/batch_informed_rrt_star.py` | 同 Informed RRT\* + 批次大小参数 | `path`：`[[x,y], …]` |

### 1.3 输出数据结构图解

```
A* 系列输出：                    RRT 系列输出：
rx = [10.0, 9.5, 9.0, ..., 0.0]   path = [
ry = [10.0, 9.8, 9.5, ..., 0.0]             [10.0, 10.0],  ← 终点
                                              [9.1,  9.7],
两个独立的 Python list               [8.3,  9.2],
可通过 zip(rx, ry) 合并               ...
                                              [0.0,  0.0]   ← 起点
                                            ]
                                   一个 list of [x, y] 列表
```

> **注意**：所有算法输出的路径均为**从终点到起点**的顺序（倒序），使用前需要 `path.reverse()` 或 `rx[::-1]`

---

## 二、路径存储格式与传递给机械臂

### 2.1 路径存储方式

当前算法输出的是**内存中的 Python 对象**，程序结束即消失。实际工程中有以下持久化方式：

#### 方式 A：保存为 JSON（推荐，轻量通用）

```python
import json

# A* 输出转 JSON
path_data = {"waypoints": list(zip(rx[::-1], ry[::-1]))}
with open("path.json", "w") as f:
    json.dump(path_data, f, indent=2)

# 读取
with open("path.json") as f:
    path_data = json.load(f)
waypoints = path_data["waypoints"]  # [(x0,y0), (x1,y1), ...]
```

#### 方式 B：保存为 CSV

```python
import csv

with open("path.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for x, y in zip(rx[::-1], ry[::-1]):
        writer.writerow([x, y])
```

#### 方式 C：ROS2 消息（直接传递，推荐用于机械臂控制）

```python
from geometry_msgs.msg import PoseArray, Pose
import rclpy

# 将路径点转为 PoseArray 发布
pose_array = PoseArray()
for x, y in zip(rx[::-1], ry[::-1]):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0  # 2D→3D 需根据实际任务设置 z
    pose_array.poses.append(pose)
```

### 2.2 从路径到机械臂运动的完整流程

```
路径规划输出                  转换层                    机械臂控制
─────────────────    ─────────────────────────    ─────────────────────
[[x,y], [x,y], …]   → 扩展到3D (x, y, z, yaw)   → MoveIt2 / 关节控制器
(任务空间路径点)      → 逆运动学 IK 求解           → UR5 驱动执行
                     → 插值生成轨迹               → 关节角度序列
```

**详细步骤说明：**

#### Step 1：坐标系转换（2D → 3D）

这些算法输出的是 2D 平面路径，UR5 工作在 3D 空间。需要根据任务类型补充第三维：

```python
def path_2d_to_3d(path_2d, z_height=0.3):
    """将2D路径扩展到3D，适用于平面抓取任务"""
    path_3d = []
    for x, y in path_2d:
        path_3d.append((x, y, z_height))
    return path_3d
```

#### Step 2：通过 MoveIt2 发送路径（ROS2 方式）

```python
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, Constraints
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient

# 逐点发送给 MoveIt2（笛卡尔路径规划）
from moveit_msgs.srv import GetCartesianPath

# 构建航点列表
waypoints = []
for x, y, z in path_3d:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0  # 末端朝向，根据任务设置
    waypoints.append(pose)

# 调用 MoveIt2 笛卡尔路径规划
request = GetCartesianPath.Request()
request.waypoints = waypoints
request.max_step = 0.01      # 插值步长 1cm
request.jump_threshold = 0.0  # 关节空间跳变阈值
```

#### Step 3：直接使用关节轨迹控制器（不经过 MoveIt2）

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

traj = JointTrajectory()
traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                    "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

for i, (q1, q2, q3, q4, q5, q6) in enumerate(joint_angles_sequence):
    point = JointTrajectoryPoint()
    point.positions = [q1, q2, q3, q4, q5, q6]
    point.time_from_start = Duration(sec=i * 1)  # 每个路径点间隔1秒
    traj.points.append(point)
```

---

## 三、A* 系列算法通俗原理

### 3.1 基础 A* — "带地图的导航员"

**一句话理解**：A* 是在格子地图上找最短路的算法，它用一张"估价表"，每次都优先走"看起来离终点最近且代价最小"的格子。

#### 核心思想

想象你在城市里开车导航：
- **已知代价 g(n)**：从起点走到当前位置，实际走了多远
- **预估代价 h(n)**：从当前位置到终点，直线距离（启发值）
- **总评分 f(n) = g(n) + h(n)**：综合评估，越小越优先探索

每次都从"待探索列表（Open Set）"中取出 f 值最小的格子，标记为"已探索（Closed Set）"，然后把它的邻居加入待探索列表——直到走到终点为止。

```
地图示例（S=起点, G=终点, #=障碍）：

S . . . .
. # # . .
. . # . .
. . . # G

A* 会找到：S→右→右→右→右下→右→G
                          ↑ 绕开了障碍物
```

#### 关键参数

| 参数 | 作用 | 典型值 |
|------|------|--------|
| `resolution` | 网格大小，越小路径越精细但越慢 | 0.5~1.0 m |
| `rr`（robot_radius）| 机器人半径，用于膨胀障碍物 | 0.1~0.5 m |
| 启发函数 | 控制"贪心程度"，常用欧式距离或曼哈顿距离 | 欧式距离 |

#### 代码调用示例

```python
from AStar.a_star import AStarPlanner

# 定义障碍物（柱子的 x, y 坐标）
ox = [0, 1, 2, 3, 4, 5, 10, 10, 10]  # 障碍物 x
oy = [0, 0, 0, 0, 0, 0, 0,  1,  2 ]  # 障碍物 y

a_star = AStarPlanner(ox, oy, resolution=1.0, rr=0.5)
rx, ry = a_star.planning(sx=0.0, sy=10.0, gx=10.0, gy=10.0)
# rx, ry 是从终点到起点的坐标序列

path = list(zip(rx[::-1], ry[::-1]))  # 反转得到起点→终点顺序
```

---

### 3.2 双向 A* — "两端同时挖隧道"

**一句话理解**：同时从起点和终点各搜索一半，两条搜索"在中间汇合"，速度比单向 A* 快约 2 倍。

#### 原理对比

```
单向 A*：  S ----→----→----→ G    搜索面积 = 全程
双向 A*：  S ----→  ←---- G    搜索面积 ≈ 一半
                   ↑汇合点
```

- 两个 Open Set 分别维护（从 S 出发的树 + 从 G 出发的树）
- 每次轮流扩展，检测两棵树是否有公共节点
- 一旦相遇就合并路径

**适用场景**：地图较大、起终点距离较远时效果明显。

---

### 3.3 Theta* — "顺直路径的 A*"

**一句话理解**：普通 A* 只能走格子的 8 个方向（45° 倍数），Theta* 会"抄近道"，沿任意角度直线走，路径更自然顺滑。

#### 核心区别

```
A* 路径（折线）：     Theta* 路径（直线段）：
S → → ↘ → → G        S ──────────────→ G
        ↑                       ↑
    必须沿格子方向           可以穿越对角线
```

Theta* 在每次更新节点父节点时，会检查"能否直接连线到祖父节点"（Line-of-Sight 检测）。如果视线不被障碍物遮挡，就直接跳过中间节点，形成更短的直线段。

**输出特点**：路径点数量更少，每段路径更长，更适合机械臂末端执行器的轨迹跟踪。

---

### 3.4 双向 A*（两侧搜索） — "更智能的相向搜索"

**文件**：`AStar/a_star_searching_from_two_side.py`

在标准双向 A* 基础上，引入了更精确的汇合条件判断，避免两侧搜索过早停止导致路径不最优的问题。

---

### 3.5 A* 变体集 — "可切换启发函数"

**文件**：`AStar/a_star_variants.py`

将多种启发函数打包在一起，方便对比实验：

| 启发函数 | 公式 | 特点 |
|----------|------|------|
| **欧式距离** | $h = \sqrt{dx^2 + dy^2}$ | 最准确，适合连续空间 |
| **曼哈顿距离** | $h = |dx| + |dy|$ | 只允许上下左右移动时最优 |
| **切比雪夫距离** | $h = \max(|dx|, |dy|)$ | 允许对角线移动时准确 |
| **加权启发** | $h = w \cdot \sqrt{dx^2 + dy^2}$ | $w>1$ 时更贪心，速度快但可能不最优 |

---

### 3.6 Hybrid A* — "考虑车头朝向的 A*"

**一句话理解**：专门为有方向约束的运动体设计（如汽车只能前进/后退，不能横向平移），搜索空间从 2D (x,y) 扩展到 3D (x, y, yaw)。

#### 核心概念

普通 A* 的节点只有位置 `(x, y)`，Hybrid A* 的节点是 `(x, y, yaw)`——同一个位置朝不同方向算不同状态。

扩展节点时使用 **Reeds-Shepp 曲线**（可前进也可后退的圆弧组合）生成候选动作，而不是简单的 8 方向移动。

```
普通 A*：节点 = (x, y)        Hybrid A*：节点 = (x, y, θ)
同一格子只有1种状态            同一格子有 N 种朝向状态
```

**对 UR5 的意义**：虽然 UR5 不是车，但 Hybrid A* 的思想可以扩展到机械臂的关节空间搜索，每个节点代表一组关节角度而不仅仅是末端位置。

---

### 3.7 时空 A*（Space-Time A*）— "考虑时间的 A*"

**一句话理解**：在 A* 的三维空间 `(x, y, t)` 中搜索，`t` 代表时间步。用于有**动态障碍物**（会移动的物体）的场景。

#### 核心思想

```
普通 A*：                 时空 A*：
地图是静态的              障碍物在不同时刻位于不同位置
只需规划一条路径           规划的路径是"在哪个时刻到达哪里"

   t=0  t=1  t=2            在 t=0 时障碍物在 (3,3)
S   .    .    G              在 t=1 时障碍物移到 (4,3)
  障碍物不动               算法会选择"等一步再走"
```

**适用场景**：多机器人协作、有移动障碍物的仓储环境。

---

## 四、RRT 系列算法通俗原理

### 4.1 基础 RRT — "随机生长的树"

**一句话理解**：RRT 就像一棵在地图里随机生长的树——从起点出发，每次随机撒一个点，然后把树枝朝那个方向延伸一小步，最终某根树枝碰到终点为止。

#### 生长过程图解

```
第1步：起点 S 是树根
第2步：随机撒点 ×，找树上最近的节点，向 × 延伸一步
第3步：重复直到碰到目标区域

地图演化示意：
[S]              [S─┐]           [S─┬──]
                    └─×              ├──×
                                     └──×
                  树慢慢长出去，最终触碰到 G
```

#### 关键参数

| 参数 | 含义 | 推荐值 |
|------|------|--------|
| `expand_dis` | 每次延伸的步长 | 地图尺寸的 5~10% |
| `goal_sample_rate` | 随机点"直接取目标点"的概率 | 5~20% |
| `max_iter` | 最大迭代次数 | 500~2000 |
| `path_resolution` | 碰撞检测精度 | 0.1~0.5 m |

#### 代码调用示例

```python
from RRT.rrt import RRT

obstacle_list = [
    (5, 5, 1),   # (x, y, 半径)
    (3, 6, 2),
    (3, 8, 2),
]

rrt = RRT(
    start=[0, 0],
    goal=[6, 10],
    rand_area=[-2, 15],
    obstacle_list=obstacle_list,
    max_iter=500,
)
path = rrt.planning(animation=False)
path.reverse()  # 转换为起点→终点顺序
# path = [[0,0], [0.5, 0.3], ..., [6,10]]
```

---

### 4.2 RRT + 路径平滑 — "修剪多余折点"

**文件**：`RRT/rrt_with_pathsmoothing.py`

**一句话理解**：RRT 找到的路径往往"锯齿状"（因为是随机采样的），路径平滑就是事后把能"抄近道"的地方连成直线，减少不必要的折点。

#### 平滑算法原理（快捷方式法）

```python
# 伪代码
path = rrt.planning()
smoothed = [path[0]]            # 从起点出发
anchor = 0                       # 当前"锚点"索引
for i in range(1, len(path)):
    # 如果从锚点到 path[i] 的直线不穿越障碍物
    if no_collision(path[anchor], path[i]):
        continue                 # 跳过中间点
    else:
        smoothed.append(path[i-1])  # 把前一个点加入
        anchor = i - 1
smoothed.append(path[-1])        # 加入终点
```

**效果**：路径点从几十个减少到十几个，更适合机械臂的轨迹跟踪。

---

### 4.3 RRT-Sobol — "更均匀的随机树"

**文件**：`RRT/rrt_with_sobol_sampler.py`

**一句话理解**：普通 RRT 用伪随机数采样，可能出现局部密集、局部空白的情况。Sobol 序列是一种"低差异序列"，保证采样点在空间中分布更均匀，让树生长得更全面。

#### 伪随机 vs 准随机

```
伪随机 (random.uniform)：   Sobol 序列：
× × ×   .   ×              × . × . × .
. × . × .   ×              . × . × . ×
× . . ×  ×  .              × . × . × .
  局部有聚集                 均匀铺满空间
```

**结果**：相同迭代次数下，Sobol RRT 的路径更短、找到路径的成功率更高，且结果可复现（确定性序列）。

---

### 4.4 RRT* — "会自我优化的 RRT"

**一句话理解**：RRT 找到一条路就不管了；RRT* 在找路的同时，不断"重走"已有路径，把绕路的地方优化掉，最终收敛到最优路径。

#### RRT vs RRT* 的核心区别

RRT* 比 RRT 多了两个步骤：

**① 选最优父节点**（Choose Parent）
- 普通 RRT：新节点的父亲就是树上最近的那个节点
- RRT*：在一个邻域圆圈内找所有节点，选择使"总代价最小"的那个作为父亲

**② 重连（Rewire）**
- 新节点加入后，检查圆圈内的其他节点：如果"经过新节点到达它们"比现在的路径更短，就更新父节点

```
RRT（找到路就停）：           RRT*（持续优化）：
S──A──B──G（代价=10）        S──A──B──G
              ↑ 随着迭代增加会找到：
              S────────G（代价=6）
```

**代价**：每次插入节点需要检查邻域，时间复杂度略高，但路径质量显著提升。

---

### 4.5 Informed RRT* — "聪明地缩小搜索范围"

**一句话理解**：RRT* 找到第一条路径后，后续采样仍然是全地图随机。Informed RRT* 说："我已经知道路径不超过长度 c，那比这条路更长的区域我完全不用去"——用一个椭圆把采样范围缩小到"有希望改进的区域"。

#### 椭圆采样原理

一旦找到代价为 $c_{best}$ 的路径，之后只在以下椭圆内采样：

$$\text{椭圆范围} = \{x \mid d(start, x) + d(x, goal) \leq c_{best}\}$$

```
全局随机采样（RRT*）：        椭圆内采样（Informed RRT*）：
×  ×  ×  ×  ×  ×           . . . . . . . .
×  ×  ×  ×  ×  ×           . . ╔══════╗ .
×  × [S─────G] ×           . . ║S────G║ .
×  ×  ×  ×  ×  ×           . . ╚══════╝ .
×  ×  ×  ×  ×  ×           . . . . . . . .
大量采样浪费在无效区域            集中精力优化最优路径
```

**效果**：收敛速度比 RRT* 快 2~5 倍（在有解的情况下）。

---

### 4.6 Batch Informed RRT* — "批量采样加速"

**一句话理解**：在 Informed RRT* 基础上，每次不是加入一个随机样本，而是批量加入一批，用图论中的"最小生成树"方法一次处理多个节点，进一步提升效率。

#### 与 Informed RRT* 的区别

```
Informed RRT*：          Batch Informed RRT*：
一次加一个点              一次加 N 个点（一批）
逐步改善路径              批次内找最优连接，改善更快

效率：★★★★              效率：★★★★★
实现复杂度：中             实现复杂度：高
```

---

## 五、算法横向对比表

### 5.1 A* 系列对比

| 算法 | 搜索方式 | 路径最优性 | 速度 | 路径平滑度 | 特殊能力 |
|------|----------|-----------|------|-----------|---------|
| **A\*** | 单向网格 | ✅ 最优 | ★★★ | ★★ | 基础通用 |
| **双向 A\*** | 双向网格 | ✅ 最优 | ★★★★ | ★★ | 长距离加速 |
| **Theta\*** | 单向任意角 | ✅ 近似最优 | ★★★ | ★★★★ | 路径更顺滑 |
| **两侧 A\*** | 双向精确 | ✅ 最优 | ★★★★ | ★★ | 精确双向 |
| **Hybrid A\*** | 3D (x,y,yaw) | ✅ 近似最优 | ★★ | ★★★★★ | 有朝向约束的运动体 |
| **时空 A\*** | 3D (x,y,t) | ✅ 最优 | ★★ | ★★ | 动态障碍物 |

### 5.2 RRT 系列对比

| 算法 | 路径最优性 | 收敛速度 | 适合高维 | 路径质量 | 特殊优势 |
|------|-----------|---------|---------|---------|---------|
| **RRT** | ❌ 概率完备 | ★★★★★ | ✅ | ★★ | 快速验证可行性 |
| **RRT+平滑** | ❌ 概率完备 | ★★★★★ | ✅ | ★★★ | 减少折点 |
| **RRT-Sobol** | ❌ 概率完备 | ★★★★★ | ✅ | ★★★ | 均匀覆盖，可复现 |
| **RRT\*** | ✅ 渐近最优 | ★★★ | ✅ | ★★★★ | 路径质量高 |
| **Informed RRT\*** | ✅ 渐近最优 | ★★★★ | ✅ | ★★★★★ | 快速收敛到最优 |
| **Batch Informed RRT\*** | ✅ 渐近最优 | ★★★★★ | ✅ | ★★★★★ | 批量高效 |

### 5.3 A* vs RRT 如何选择

```
┌─────────────────────────────────────────┐
│         你的路径规划任务是什么？           │
└──────────────────┬──────────────────────┘
                   │
         ┌─────────┴──────────┐
         ▼                    ▼
  地图是离散网格？          连续高维空间？
  障碍物已知？              障碍物稀疏？
         │                    │
  ✅ 用 A* 系列          ✅ 用 RRT 系列
         │                    │
  需要方向约束？           需要最优解？
  (如有轮子的小车)         迭代时间充足？
         │                    │
  ✅ Hybrid A*          ✅ RRT* / Informed RRT*
         │
  有动态障碍物？
         │
  ✅ Space-Time A*
```

---

## 六、UR5 实际对接指南

### 6.1 2D 路径规划在 UR5 上的定位

本项目中的路径规划算法均基于 **2D 平面**，与 UR5（6自由度机械臂）的关系如下：

```
算法层（本项目）        中间层（需自行实现）         执行层
──────────────        ──────────────────         ──────────────
2D 平面路径点    →    末端3D位姿规划          →   UR5 关节空间
[[x,y], ...]         [x, y, z, roll, pitch, yaw] → [q1..q6]
                      ↑ 逆运动学 (IK)
                      MoveIt2 自动完成
```

### 6.2 使用 MoveIt2 执行路径的完整代码框架

```python
#!/usr/bin/env python3
"""
将路径规划输出转换为 UR5 MoveIt2 笛卡尔轨迹并执行
需要 ROS2 Humble + MoveIt2 环境
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Pose

# 路径规划部分（使用本项目算法）
import sys
sys.path.insert(0, "/workspaces/ur5/PathPlanning")
from RRT.rrt import RRT
from AStar.a_star import AStarPlanner


def plan_path_rrt(start, goal, obstacles):
    """用 RRT 规划 2D 路径"""
    rrt = RRT(
        start=start, goal=goal,
        rand_area=[-1, 11],
        obstacle_list=obstacles,
        max_iter=1000
    )
    path = rrt.planning(animation=False)
    if path:
        path.reverse()  # 转为起点→终点
    return path


def plan_path_astar(start, goal, obstacles_xy, resolution=0.5):
    """用 A* 规划 2D 路径"""
    ox = [pt[0] for pt in obstacles_xy]
    oy = [pt[1] for pt in obstacles_xy]
    planner = AStarPlanner(ox, oy, resolution=resolution, rr=0.3)
    rx, ry = planner.planning(start[0], start[1], goal[0], goal[1])
    return list(zip(rx[::-1], ry[::-1]))


class UR5PathExecutor(Node):
    def __init__(self):
        super().__init__("ur5_path_executor")
        self.cart_path_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

    def path_2d_to_poses(self, path_2d, z=0.3, orientation_w=1.0):
        """将 2D 路径点转换为 3D Pose 列表"""
        poses = []
        for x, y in path_2d:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.w = orientation_w  # 末端保持垂直向下
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            poses.append(p)
        return poses

    def execute_cartesian_path(self, path_2d):
        """通过 MoveIt2 执行笛卡尔路径"""
        poses = self.path_2d_to_poses(path_2d)

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "manipulator"
        req.waypoints = poses
        req.max_step = 0.01          # 每段最大插值长度 1cm
        req.jump_threshold = 0.0     # 关闭关节跳变检测
        req.avoid_collisions = True

        future = self.cart_path_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        self.get_logger().info(
            f"笛卡尔路径规划完成度: {result.fraction:.1%}, "
            f"路径点数: {len(result.solution.joint_trajectory.points)}"
        )
        return result.solution  # RobotTrajectory 对象


def main():
    rclpy.init()

    # 1. 定义障碍物（工作台上的物体）
    obstacles = [(3.0, 5.0, 0.5), (7.0, 4.0, 0.8)]  # (x, y, radius)

    # 2. 路径规划（选择算法）
    path = plan_path_rrt(
        start=[0.2, 0.2],
        goal=[0.8, 0.8],
        obstacles=obstacles
    )

    if path is None:
        print("路径规划失败！")
        return

    print(f"规划路径点数: {len(path)}")
    for i, (x, y) in enumerate(path):
        print(f"  点{i}: ({x:.3f}, {y:.3f})")

    # 3. 执行（需要 ROS2 + MoveIt2 环境）
    executor = UR5PathExecutor()
    trajectory = executor.execute_cartesian_path(path)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 6.3 MoveIt2 规划器选择建议

| 场景 | 推荐算法 | MoveIt2 规划器 |
|------|---------|----------------|
| 已知障碍物，精确路径 | A\* / Theta\* | OMPL + 笛卡尔路径 |
| 高维关节空间，障碍物稀疏 | RRT\* / Informed RRT\* | OMPL RRTstar |
| 实时在线规划 | RRT（快速） | OMPL RRT |
| 多机器人/动态障碍 | 时空 A\* | 自定义规划器 |
| 末端姿态有严格要求 | Hybrid A\* | OMPL + IK |

### 6.4 路径规划 → UR5 完整数据流

```
┌──────────────────────────────────────────────────────────────┐
│                        数据流总览                              │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 输入定义                                                  │
│     • 工作空间障碍物位置（从传感器/已知地图获取）                  │
│     • 起点 = UR5 当前末端坐标（从 /joint_states 正运动学获取）   │
│     • 终点 = 目标抓取/放置位置                                  │
│                                                              │
│  2. 路径规划（本项目算法）                                       │
│     • 输出：[[x0,y0], [x1,y1], ..., [xn,yn]]                │
│                                                              │
│  3. 坐标系转换                                                 │
│     • 2D (x,y) → 3D (x,y,z) + 末端朝向四元数                 │
│     • 参考系：base_link                                       │
│                                                              │
│  4. MoveIt2 笛卡尔路径规划                                     │
│     • 输入：3D Pose 序列                                       │
│     • 输出：关节轨迹 JointTrajectory                            │
│     • 内部自动完成：IK + 碰撞检测 + 插值                        │
│                                                              │
│  5. 执行                                                      │
│     • 发布到 /scaled_joint_trajectory_controller             │
│     • UR5 驱动器接收并执行关节运动                               │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 附录：快速参考卡片

```
A* 调用：
  planner = AStarPlanner(ox, oy, resolution, rr)
  rx, ry = planner.planning(sx, sy, gx, gy)
  path = list(zip(rx[::-1], ry[::-1]))  # ← 别忘了反转！

RRT 调用：
  rrt = RRT(start, goal, rand_area, obstacle_list)
  path = rrt.planning(animation=False)
  path.reverse()  # ← 别忘了反转！

路径格式统一化：
  # A* → 统一格式
  path = [(x, y) for x, y in zip(rx[::-1], ry[::-1])]

  # RRT → 统一格式
  path.reverse()
  path = [(pt[0], pt[1]) for pt in path]

保存路径：
  import json
  json.dump({"path": path}, open("path.json","w"))

读取路径：
  path = json.load(open("path.json"))["path"]
```
