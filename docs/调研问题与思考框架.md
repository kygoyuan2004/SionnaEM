# Micro-Doppler 集成到 Sionna RT 调研

## 师兄布置的核心问题

**Micro-Doppler 能否集成到 Sionna RT 中？如果能，怎么加？**

具体拆解为三个子问题：

1. **公式层面的区别**：标准 Doppler 和 Micro-Doppler 在计算上有什么不同？
2. **代码层面的定位**：这个区别在 Sionna 代码中对应哪个位置？改动点在哪里？
3. **可行性判断**：是否像师兄预期的那样——"只是公式的简单区别，比较好找到位置直接加"？

---

## 调研时应带着的思考框架

### 1. 先理解"现有逻辑"再谈"加什么"

Sionna 当前的流程是：**先射线追踪 → 得到路径参数 (a, tau, angles) → 加上电磁系数 → 生成 CIR**。

要搞清楚这个流程的每个环节在代码中对应哪个文件、哪个函数。只有把现有逻辑吃透了，才能判断 micro-Doppler 应该"嵌"在哪个环节。

关键文件：
- `path_solvers/paths.py` — Paths 类，存储路径数据，包含 `cir()`、`cfr()` 方法
- `path_solvers/field_calculator.py` — FieldCalculator，计算电磁场系数和延迟
- `path_solvers/paths_buffer.py` — PathsBuffer，路径计算时的缓冲区
- `path_solvers/path_solver.py` — PathSolver，路径求解入口

### 2. 抓住"公式差异"这个核心

标准 Doppler：频移是**常数** → 相位是线性斜坡：

```
a_i^b(t) = a_i * e^{-j2πfτ_i} * e^{j2π f_Δ t}
```

其中 f_Δ 由 TX/RX 速度和场景物体速度决定，对每条路径固定。

Micro-Doppler：频移是**时变的**（由旋转叶片产生周期性调制）→ 相位多了正弦调制项：

```
φ(t) = 2π f_Δ t + β * sin(2π f_rot t + φ₀)
```

其中 β = (4πR/λ) * cos(θ) 是调制指数，f_rot 是叶片旋转频率。

调研时要能**用公式讲清楚**这个差异，最好能写出推导过程。

### 3. 区分"物理模型"和"计算实现"

- **物理上**：micro-Doppler 需要建模 UAV 的旋转叶片散射点，不是简单的点目标。每个叶片上的散射点具有时变的位置和速度。
- **实现上**：在 Sionna 中可能不需要改动射线追踪引擎本身，而是在"路径 → CIR"的转换阶段加调制。射线追踪只需处理好静态场景，旋转部件的贡献可以通过后处理叠加。
- 调研时要清楚哪些是物理假设的简化，哪些是代码改动的范围。

### 4. 带着"向何老师汇报"的意识

何老师关心的是：**这个方向能不能做，难度多大，改动的范围在哪**。

你的调研结论要能回答：
- 可以 / 不可以，原因是……
- 如果可以，改动在 X 文件 Y 函数，大致工作量……
- 需要新增什么参数（如叶片数量、旋转频率、半径等）
- 是否需要修改射线追踪引擎，还是只需修改 CIR 生成部分

### 5. 两篇论文的分工

- **Micro-Doppler Signature Simulation of Multirotor UAVs Using Ray Tracing**：偏方法，讲如何用射线追踪仿真多旋翼 UAV 的 micro-Doppler 特征。重点看它的公式推导和仿真流程。
- **Micro-Doppler Signature-Based Detection, Classification and Localization of Small UAV**：偏应用，讲如何利用 micro-Doppler 做检测分类定位。重点看它的特征提取和分类思路。

---

## 调研产出建议

1. 标准 Doppler 与 Micro-Doppler 的公式对比（带推导）
2. Sionna 当前 Doppler 计算流程的代码走读（标注改动点）
3. 集成方案（在哪改、改什么、新增什么参数）
4. 可行性结论，附工作量估计
