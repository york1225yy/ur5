# 双向 A*（Bidirectional A-Star）路径规划

## 算法简介

双向 A\* 是标准 A\* 的改进版本，**同时从起点和终点双向扩展搜索树**，当两棵树相遇时找到路径。

### 核心改进

```
标准 A*：  Start ─────────────────────► Goal
双向 A*：  Start ──────► Meet ◄──────── Goal
                          Point
```

两个方向各自维护独立的 open/closed 列表，搜索到相遇点时合并路径。
理论上搜索节点数从 $O(b^d)$ 降为 $O(2 \cdot b^{d/2})$，**速度提升约 2×**。

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `bidirectional_a_star.py` | 双向 A\* 完整实现，含可视化 |

---

## 快速使用

### 直接运行演示

```bash
python bidirectional_a_star.py
```

### 在代码中调用

```python
from bidirectional_a_star import BidirectionalAStarPlanner

# 定义障碍物
ox, oy = [], []
for i in range(0, 50):
    ox.append(i); oy.append(0.0)
for i in range(0, 50):
    ox.append(i); oy.append(30.0)
# ... 更多障碍物

# 创建规划器（接口与标准 A* 完全相同）
planner = BidirectionalAStarPlanner(ox, oy, resolution=2.0, rr=1.0)

# 执行规划
rx, ry = planner.planning(sx=10.0, sy=10.0, gx=35.0, gy=20.0)
```

---

## 关键参数（与 A* 相同）

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `resolution` | 栅格分辨率（m） | 0.5 ~ 2.0 |
| `rr` | 机器人安全半径（m） | 机器人半径 × 1.2 |

---

## 与标准 A* 对比

| 对比项 | A\* | 双向 A\* |
|--------|-----|---------|
| 搜索方向 | 单向（起 → 终） | 双向同时扩展 |
| 搜索节点数（理论） | $O(b^d)$ | $O(2b^{d/2})$ |
| 速度 | 基准 | **约 2× 更快** |
| 路径最优性 | ✅ | ✅ |
| 实现复杂度 | 低 | 中 |
| 内存占用 | 较低 | 略高（两棵树） |

---

## 适用场景

- 大型 2D 地图中末端轨迹规划（双向可显著缩短搜索时间）
- 起点与终点距离较远时优势明显
- 静态已知障碍物环境
