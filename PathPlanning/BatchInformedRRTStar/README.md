# BIT*（Batch Informed Trees）路径规划

## 算法简介

BIT\*（Batch Informed Trees）结合了**基于采样的方法**（RRT\* 系列）与 **A\* 式图搜索**的优势，通过**批量采样 + 懒惰连接（Lazy Connect）**实现目前最高效的渐进最优规划。

### 核心创新

| 特性 | 说明 |
|------|------|
| **批量采样** | 每次迭代批量采样 $m$ 个点，构建隐式随机几何图（RGG） |
| **A\* 式搜索** | 在 RGG 上使用类 A\* 方式按代价顺序扩展边 |
| **懒惰碰撞检测** | 仅在必要时才做碰撞检测，减少无效计算 |
| **Informed 采样** | 找到初解后同样限定在椭圆区域内采样 |

### 与其他算法的关系

```
RRT  ──┐
       ├─(渐进最优)─► RRT* ──┐
Dijkstra─┘                    ├─(批量+A*)─► BIT*
A*    ─────────────────────────┘
                  ↑ 兼具随机采样的完备性 + 图搜索的效率
```

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `batch_informed_rrt_star.py` | BIT\* 完整实现，含 RTree（搜索树）和 Queue（优先队列）类 |

---

## 快速使用

### 直接运行演示

```bash
python batch_informed_rrt_star.py
```

### 在代码中调用

```python
from batch_informed_rrt_star import BITStar

# 边界和起终点
x_start = (0, 0)     # 起点 (x, y)
x_goal  = (6, 10)    # 终点 (x, y)

# 障碍物列表：[x, y, radius]
obstacles = [
    [5,  5,  0.5],
    [9,  6,  1.0],
    [7,  5,  1.0],
    [1,  5,  1.0],
    [3,  5,  1.0],
    [7,  3,  1.0],
]

bit_star = BITStar(
    x_start=x_start,
    x_goal=x_goal,
    eta=2.0,                 # 扩展步长
    iter_max=200,            # 最大迭代批次数
    # 搜索边界 [xmin, xmax, ymin, ymax]
)

bit_star.planning()
```

---

## 关键参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `iter_max` | 最大迭代批次（非单步），每批次采样多个点 | 100 ~ 500 |
| `eta` | 扩展步长（影响树的粒度） | 1.0 ~ 3.0 |
| 采样批次大小 | 代码内 `batch_size` 参数，控制每批采样数 | 50 ~ 200 |

---

## 四种算法综合对比

| 算法 | 初解速度 | 收敛速度 | 实现复杂度 | 适用场景 |
|------|---------|---------|----------|---------|
| RRT | ⭐⭐⭐⭐⭐ 最快 | ❌ 不优化 | ⭐ 最简单 | 快速验证可行性 |
| RRT\* | ⭐⭐⭐⭐ | ⭐⭐ 慢 | ⭐⭐ | 标准最优规划 |
| Informed RRT\* | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ 快 | ⭐⭐⭐ | 时间有限时的优化 |
| **BIT\*** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ 最快 | ⭐⭐⭐⭐ | 追求最优效率 |

---

## 适用场景

- 计算资源充足、追求最优规划效率
- 复杂障碍物环境下的 UR5 关节空间规划
- 作为学术研究的对比算法基准
- 需要理解采样规划与图搜索结合思想

---

## 参考文献

Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2015).  
*Batch Informed Trees (BIT\*): Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs.*  
[arXiv:1405.5848](https://arxiv.org/abs/1405.5848)
