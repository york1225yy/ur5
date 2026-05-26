# Theta*（任意角度 A*）路径规划

## 算法简介

Theta\* 是 A\* 的扩展，**允许路径沿任意角度移动**（不局限于 8 方向栅格），生成的路径更加自然平滑。

### 核心差异：Line-of-Sight 检查

标准 A\* 只能沿栅格方向移动，路径呈锯齿状。  
Theta\* 在扩展节点时额外检查**当前节点的祖父节点**是否与新节点之间存在直视路径（Line-of-Sight），若有则**跳过中间节点直接连接**，实现任意角度路径。

```
A*：    Start → A → B → C → D → Goal    （锯齿）
Theta*：Start ────────→ C ────→ Goal     （任意角度，更短）
```

---

## 文件说明

| 文件 | 内容 |
|------|------|
| `theta_star.py` | Theta\* 完整实现，含 Line-of-Sight 检查与可视化 |

---

## 快速使用

### 直接运行演示

```bash
python theta_star.py
```

### 在代码中调用

```python
from theta_star import ThetaStarPlanner

# 定义障碍物
ox, oy = [], []
for i in range(-10, 60):
    ox.append(i); oy.append(-10.0)
for i in range(-10, 60):
    ox.append(60.0); oy.append(i)
# ... 更多障碍物

# 创建规划器（接口与标准 A* 完全相同）
planner = ThetaStarPlanner(ox, oy, resolution=2.0, rr=1.0)

# 执行规划
rx, ry = planner.planning(sx=0.0, sy=0.0, gx=50.0, gy=50.0)
```

---

## 控制变量

文件顶部可切换是否使用 Theta\*（对比实验用）：

```python
use_theta_star = True   # True → Theta*，False → 退化为标准 A*
```

---

## 与 A* 路径对比

| 对比项 | A\* | Theta\* |
|--------|-----|---------|
| 移动方向 | 8 方向（固定角度） | **任意角度** |
| 路径平滑度 | 锯齿状 | **自然平滑** |
| 路径长度 | 偏长（绕格子走） | **更短** |
| 计算量 | 低 | 略高（Line-of-Sight 检查） |
| 最优性 | 栅格最优 | **欧式空间近最优** |

---

## 适用场景

- 需要自然平滑轨迹的末端路径规划
- 对路径长度敏感的场景（节省执行时间）
- 静态障碍物、2D/3D 工作空间
- 直接作为末端执行器笛卡尔轨迹输入 MoveIt2
