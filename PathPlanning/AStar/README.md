# A*（A-Star）路径规划

## 算法简介

A\* 是一种**启发式最优路径搜索算法**，在栅格地图中寻找从起点到终点代价最小的路径。

### 评估函数

$$f(n) = g(n) + h(n)$$

| 项 | 含义 |
|----|------|
| $g(n)$ | 从起点到当前节点 $n$ 的实际累计代价 |
| $h(n)$ | 从节点 $n$ 到终点的启发式估计代价（欧氏距离） |
| $f(n)$ | 综合优先级，越小越优先扩展 |

当 $h(n)=0$ 退化为 Dijkstra；当 $h(n)$ 为完美估计时速度最快。

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `a_star.py` | 标准 A\* 算法，8 方向移动，完整可视化 |
| `a_star_searching_from_two_side.py` | 双向 A\*（Two-Side A\*），从起点和终点同时扩展，搜索效率约提升 2× |
| `a_star_variants.py` | A\* 变体：加权 A\*（Weighted A\*）等 |

---

## 快速使用

### 直接运行演示

```bash
# 运行标准 A*（含 matplotlib 动画）
python a_star.py

# 运行双向 A*（含动画，随机生成障碍物）
python a_star_searching_from_two_side.py
```

运行后会弹出动画窗口，绿色圆圈为起点，蓝色叉为终点，红色线为规划路径。

### 双向 A\* 调用说明（a_star_searching_from_two_side.py）

双向 A\* 同时从起点和终点向中间扩展，当两棵树的扩展前沿相交时即找到路径。

```python
# 直接运行，程序自动随机生成障碍物并演示
python a_star_searching_from_two_side.py

# 核心函数
from a_star_searching_from_two_side import searching_from_two_side

# path: 路径坐标列表；visited: 已访问节点列表
path, visited = searching_from_two_side(
    m,          # 地图（二维数组，0=自由，1=障碍）
    start,      # 起点坐标 [row, col]
    end         # 终点坐标 [row, col]
)
```

**与标准 A\* 对比**：

| 指标 | 标准 A\* | 双向 A\* |
|------|---------|---------|
| 搜索方向 | 单向（起→终） | 双向（起+终同时） |
| 搜索节点数 | $O(b^d)$ | $O(2 \times b^{d/2})$ |
| 速度（对称地图） | 基准 | 约 **2×** 更快 |
| 实现复杂度 | 低 | 中（需检测两树交汇） |

### 在代码中调用（标准 A\*）

```python
from a_star import AStarPlanner

# 定义障碍物坐标列表（米）
ox, oy = [], []
for i in range(0, 50):
    ox.append(i)
    oy.append(0.0)       # 下边界
for i in range(0, 50):
    ox.append(i)
    oy.append(30.0)      # 上边界
# ... 添加更多障碍物

# 创建规划器
#   ox, oy      : 障碍物 x/y 坐标列表 [m]
#   resolution  : 栅格分辨率 [m]，越小越精确但越慢
#   rr          : 机器人半径 [m]，用于安全膨胀
planner = AStarPlanner(ox, oy, resolution=2.0, rr=1.0)

# 执行路径规划
#   sx, sy : 起点坐标 [m]
#   gx, gy : 终点坐标 [m]
rx, ry = planner.planning(sx=10.0, sy=10.0, gx=50.0, gy=50.0)

# rx, ry 为规划路径的 x/y 坐标序列（从终点到起点倒序）
print("路径点数:", len(rx))
```

---

## 关键参数说明

| 参数 | 类型 | 说明 | 推荐值 |
|------|------|------|--------|
| `resolution` | float | 栅格分辨率（m），控制路径精度与速度 | 0.5 ~ 2.0 |
| `rr` | float | 机器人半径（m），障碍物安全膨胀距离 | 机器人实际半径 × 1.2 |
| `show_animation` | bool | 是否显示实时规划动画 | 调试时 True，集成时 False |

---

## 运动模型

默认 **8 方向**移动（上下左右 + 四个对角线），对角线移动代价为 $\sqrt{2}$：

```python
# 运动模型：[dx, dy, cost]
motion = [
    [1,  0,  1],       # 右
    [0,  1,  1],       # 上
    [-1, 0,  1],       # 左
    [0, -1,  1],       # 下
    [-1,-1,  math.sqrt(2)],  # 左下
    [-1, 1,  math.sqrt(2)],  # 左上
    [1, -1,  math.sqrt(2)],  # 右下
    [1,  1,  math.sqrt(2)],  # 右上
]
```

---

## 算法复杂度

| 指标 | 值 |
|------|-----|
| 时间复杂度 | $O(b^d)$，$b$ 为分支因子，$d$ 为深度 |
| 空间复杂度 | $O(b^d)$（需存储 open/closed 列表） |
| 最优性 | ✅ 保证最优（$h$ 可接受时） |
| 完备性 | ✅ 保证找到路径（若路径存在） |

---

## 适用场景与限制

**适用**：
- 2D 工作平面内的末端执行器路径规划
- 已知静态障碍物的离散栅格地图
- 需要保证最优路径的场景

**不适用**：
- 高维连续关节空间（6DOF → 内存爆炸）
- 动态障碍物实时变化（建议用 D\* Lite）
- 超大地图（建议配合分层规划）
