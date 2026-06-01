# UAV 微多普勒 MDS 数据生成链路系统评估

> 生成日期：2026-06-01  
> 项目：SionnaEM  
> 评估对象：Blender 建模 -> Sionna RT -> UAV 动态散射体 -> CIR/CFR/复信号 -> STFT -> MDS

## 0. 核心结论

你的总体想法是可行的，但需要把“雷达反射问题”和“Sionna RT 通信信道问题”严格区分。最稳妥的论文表述不是“Sionna RT 原生仿真了完整旋翼雷达散射”，而是：

> 本项目利用 Blender/Sionna RT 建模静态传播环境和多径路径结构，在 UAV 目标处引入机身与旋翼叶片散射点模型，通过每帧 RT 重算或 CIR 后处理相位调制生成相干复基带序列，再由 STFT 得到 UAV micro-Doppler spectrogram。该方法在点散射体、远场、小目标、路径结构慢变等假设下，能够生成物理可解释且工程可用的 MDS。

结论分三层：

| 层次 | 严谨程度 | 推荐用途 |
|------|----------|----------|
| 理论严格模型 | 需要 Tx -> UAV 表面/叶片 -> Rx 的单基地或双基地雷达散射、RCS、材料、姿态、遮挡与多径全建模 | 高保真雷达散射论文 |
| Sionna RT 可实现模型 | 可加载 Mitsuba XML 场景、计算 TX/RX 间路径、CIR/CFR、标准 Doppler；可逐帧移动 TX/RX/点对象重算路径 | 小规模验证、对比实验 |
| 工程可接受模型 | 静态环境 RT + UAV 点散射体解析相位调制 + 相干路径叠加 + STFT | 数据生成、组会、方法论文初稿 |

本项目当前最成熟的路线是：

```text
Blender/程序化静态场景
  -> Sionna RT PathSolver 求静态多径 CIR
  -> MicroDopplerModulator 给机身和叶片散射点加入相干相位调制
  -> sum paths 得到复基带时间序列
  -> STFT 生成 MDS
```

同时，项目中已经有“逐 snapshot 更新点散射体并重跑 PathSolver”的脚本，可作为更接近 RT 几何的交叉验证路线。

## 1. 你的思路是否存在明显疏漏

### 1.1 物理建模

UAV MDS 的本质不是“每帧得到一个路径损耗”，而是“连续时间复回波的相干相位变化”。因此必须保存复数 CIR/CFR 或复基带信号，不能只保存 pathloss、RSS、radio map。

对第 k 个散射点，单基地近似下：

$$
s_k(t)=A_k(t)\exp\left(-j\frac{4\pi}{\lambda}R_k(t)\right)
$$

双基地近似下：

$$
s_k(t)=A_k(t)\exp\left(-j\frac{2\pi}{\lambda}
\left(R_{\mathrm{tx},k}(t)+R_{k,\mathrm{rx}}(t)\right)\right)
$$

其中 $A_k(t)$ 可包含 RCS、天线方向图、传播损耗、材料反射损耗和多径权重。MDS 来自 $s(t)=\sum_k s_k(t)$ 的相位随时间变化，而不是来自功率随时间变化。

### 1.2 Sionna RT 能力边界

Sionna RT 官方文档说明，`PathSolver` 返回 `Paths`，`paths.cir()` 可生成 CIR，且 CIR 的时间演化可基于每条路径 Doppler 相位：

$$
a_i^b(t)=a_i e^{-j2\pi f\tau_i} e^{j2\pi f_{\Delta,i}t}
$$

这适合标准 Doppler 或给对象设置速度后的慢速运动。官方技术报告也明确指出，移动物体可以逐步移动并每步重算路径，这最准确但计算昂贵；也可以给对象速度并累积路径 Doppler，这更快但依赖路径结构不变的假设。

对高速旋翼，Sionna RT 的原生速度/Doppler 模型不能自动理解“叶片绕电机旋转导致不同叶片点具有周期速度场”。因此有两种可行实现：

1. 显式逐帧更新叶片点散射体位置，重跑 `PathSolver`。
2. 静态 RT 求环境路径，再用解析散射点模型对 CIR 系数做相位调制。

第二种是本项目当前主线，更适合批量生成数据。

### 1.3 UAV 散射体建模

你的“机身 + 叶片散射点”建模方向合理，但必须承认它不是完整电磁散射。它忽略了：

- 叶片真实形状、姿态、弯曲、厚度与材料；
- 叶片 RCS 随入射角、极化、频率、转角变化；
- 机身遮挡叶片、叶片遮挡叶片；
- 叶片沿半径方向不同点的不同径向速度；
- 多旋翼间相位、转向和转速不一致。

不过用于 MDS 结构验证和数据生成，这个近似是合理的，尤其是先做单点、双叶片、多叶片逐级验证时。

### 1.4 单基地与双基地链路

这是最容易混淆的点。Sionna RT 本质上是通信链路 `Transmitter -> Receiver` 的传播求解器；雷达目标回波是 `Tx -> target scatterer -> Rx` 的散射链路。

如果用“叶片点作为 TX，雷达作为 RX”，得到的是 one-way 通信型相位：

$$
\phi(t)=-\frac{2\pi}{\lambda}R(t)
$$

单基地雷达应是 round-trip 相位：

$$
\phi(t)=-\frac{4\pi}{\lambda}R(t)
$$

因此 one-way RT 结果直接当单基地雷达会少一个约 2 倍的 Doppler/相位因子。项目里的 `rt_micro_doppler_vs_standard_doppler_single.py` 已经明确标注：它是 one-way RT CIR-based signal，不是完整 monostatic radar round-trip model。

### 1.5 标准 Doppler 与 micro-Doppler 叠加

机身平动产生中心频移，旋翼旋转产生围绕中心频移的周期展宽。若目标整体速度为 $\mathbf{V}$，叶片相对机身位置为 $\mathbf{r}$，角速度为 $\boldsymbol{\omega}$，视线单位方向为 $\mathbf{n}$，单基地近似下：

$$
f_D \approx \frac{2}{\lambda}\mathbf{V}\cdot\mathbf{n}
$$

$$
f_{mD}(t) \approx \frac{2}{\lambda}
\left(\boldsymbol{\omega}\times\mathbf{r}(t)\right)\cdot\mathbf{n}
$$

总瞬时频率近似为：

$$
f_{\mathrm{inst},k}(t)=f_{D,\mathrm{body}}+f_{mD,k}(t)
$$

若是 one-way 通信链路，前面的系数从 $2/\lambda$ 变成 $1/\lambda$。若是双基地，$\mathbf{n}$ 要替换为发射方向和接收方向的双基地几何和。

### 1.6 相干相位

MDS 必须由相干复信号得到。常见错误是只把每帧的路径功率、路径损耗或幅度拿来做 STFT，这样得到的是幅度闪烁谱，不是 Doppler 谱。

正确做法是保存：

```python
a_dynamic[t, rx, tx, tx_ant, path, rx_ant]  # complex
signal[t] = np.sum(a_dynamic[t, 0, 0, 0, :, 0])
```

或者从 CFR 中选定一个/多个子载波的复数 $H[k,t]$ 做 STFT。若跨路径、跨散射点、跨快照的相位参考不一致，MDS 会失真。

### 1.7 采样率与相位混叠

微多普勒峰值频偏近似：

$$
f_{mD,\max}\approx \frac{2}{\lambda}v_{\mathrm{tip}}\cos\theta
=\frac{2}{\lambda}(2\pi R f_{\mathrm{rot}})\cos\theta
$$

以项目常用参数 $f_c=28$ GHz, $R=0.15$ m, $f_{\mathrm{rot}}=100$ Hz 为例：

$$
\lambda\approx10.7\text{ mm},\quad
v_{\mathrm{tip}}\approx94.2\text{ m/s},\quad
f_{mD,\max}\approx17.6\text{ kHz}
$$

所以 snapshot rate 至少应满足：

$$
f_s > 2(f_{D,\max}+f_{mD,\max})
$$

工程上建议留 1.5 到 2 倍裕量，即 50 kHz 到 100 kHz。项目中 50 kHz 是合理下限；2 kHz 或 4 kHz 只能用于低转速/小半径缩放验证，不能用于默认 28 GHz、0.15 m、100 Hz 的完整谱图。

### 1.8 RCS 与材料建模

Sionna RT 的 radio material 能描述环境物体的相对介电常数、导电率、厚度、镜面/漫反射等，但它不会自动把一个 UAV 网格变成高保真雷达目标 RCS 库。若论文强调“雷达散射真实度”，需要额外的 RCS 模型或测量/电磁仿真标定。

工程近似中可以把散射点权重 $A_k$ 当成相对 RCS：

- 机身：`body_amplitude = 1.0`
- 叶片：`blade_amplitude = 0.05` 到 `0.2`
- 可选：$A_k(t)=A_0|\cos\alpha_k(t)|^p$ 模拟叶片闪烁

### 1.9 多径环境

多径会带来：

- 同一旋翼调制的延迟副本；
- 频谱中的弱“鬼影”；
- 由于不同路径入射/出射方向不同，微多普勒投影不同；
- 相干叠加导致的增强/抵消。

当前 `modulate_cir()` 对所有 RT 路径施加同一个复调制标量。这能模拟“环境路径结构 + 目标微多普勒”的一阶效果，但不区分每条路径的不同观测方向。更高保真版本应利用每条路径的 AoD/AoA 或等效路径方向，对每条路径计算不同 $\mathbf{n}_p$。

## 2. 工程实现是否可行

### 2.1 逐 snapshot PathSolver 路线

这条路线可跑通，项目中已有原型：

- `src/rt_micro_doppler_vs_standard_doppler.py`
- `src/rt_micro_doppler_vs_standard_doppler_single.py`

流程为：

```text
创建 Sionna 场景
  -> 添加 radar receiver
  -> 添加 body 和 blade_i 作为点 TX
  -> 每个 snapshot 更新 body/blade_i.position
  -> PathSolver(scene)
  -> paths.cir(...)
  -> 对每个 TX 的路径复系数按散射权重相干求和
  -> signal[t]
  -> STFT
```

优点：

- 相位来自每帧 RT 几何；
- 能验证单点/双叶片/四旋翼的基本几何关系；
- 可以和解析模型交叉验证。

难点：

- 计算量随 `n_snapshots * n_scatterers * max_depth` 增长；
- 当前实现是 one-way 点 TX 模型，不是完整 radar reflection；
- 点 TX 不等价于被动散射体；
- 高采样率下，例如 50 kHz、0.2 s 即 10000 帧，逐帧 RT 不现实；
- 路径数会随叶片位置变化而跳变，可能造成非物理的相位断裂。

规避方式：

- 用小半径/低转速/低采样率做 RT 原生验证；
- `max_depth=0` 或简单场景先验证；
- 批量数据生成使用 CIR 后处理调制；
- 逐帧 RT 只作为对照实验和关键帧校准。

### 2.2 静态 RT + CIR 后处理路线

这条路线更适合你的最终数据生成。项目中已有：

- `src/micro_doppler_modulator.py`
- `src/micro_doppler_integration_demo.py`
- `src/step7_dynamic_drone_demo.py` 的 Mode B
- `docs/Blender_Sionna_MicroDoppler_Chain.md`

推荐流程：

```text
Blender/程序化场景 -> XML/PLY
  -> load_scene()
  -> 添加 BS/Radar 和 drone_rx/uav_tx
  -> PathSolver 求一次静态 CIR
  -> MicroDopplerModulator 生成散射点轨迹与相位
  -> a_dynamic(t)=a_static * modulation(t)
  -> sum paths 得到复基带 signal(t)
  -> STFT 得到 MDS
```

优点：

- 快，适合批量生成；
- 相干相位可控；
- 方便做参数扫描；
- 与理论公式对应清楚；
- 不修改 Sionna RT 源码。

主要近似：

- 散射点共享静态多径路径结构；
- 每条路径使用同一个微多普勒调制，路径方向差异未完全建模；
- 叶片只是点散射体，不是真实网格/RCS；
- 单基地雷达需要显式 round-trip 相位因子。

## 3. 生成的 MDS 是否足够精准

### 3.1 物理上更真实的模型

更真实模型应包含：

- UAV 机身、机臂、旋翼完整网格；
- 真实材料参数和频率相关 RCS；
- 叶片旋转姿态、厚度、翼型和遮挡；
- Tx -> UAV -> Rx 的反射链路；
- 单基地或双基地几何；
- 环境多径和目标散射多径耦合；
- 宽带波形、距离门、脉冲重复频率、相干处理间隔；
- 天线方向图、极化、噪声、杂波。

这不是 Sionna RT 直接开箱就能完整完成的，需要额外雷达散射建模。

### 3.2 工程上近似可用的模型

当前项目的模型用于以下目标是可信的：

- 验证标准 Doppler 和 micro-Doppler 的公式差异；
- 生成具有正确峰值频偏、边带间隔、旋翼周期、机身 Doppler 线的 MDS；
- 做 UAV 有无检测、旋翼参数敏感性、单旋翼/四旋翼形态对比；
- 在组会或方法论文中说明“RT 环境 + 解析微多普勒散射点”的混合仿真链路。

但不应声称：

- 已精确复现真实 UAV RCS；
- 已完整模拟单基地雷达电磁散射；
- 叶片网格在 Sionna 中真实高速旋转并产生了全波级散射；
- MDS 幅度可直接对标实测雷达回波绝对功率。

## 4. 标准 Doppler 与 micro-Doppler 如何同时建模

设第 k 个散射点位置为：

$$
\mathbf{x}_k(t)=\mathbf{x}_0+\mathbf{V}t+\mathbf{R}_j+
\mathbf{Q}(t)\mathbf{r}_{k,0}
$$

其中 $\mathbf{x}_0+\mathbf{V}t$ 是 UAV 机身平动，$\mathbf{R}_j$ 是第 j 个旋翼中心相对机身的偏移，$\mathbf{Q}(t)$ 是旋转矩阵。速度为：

$$
\dot{\mathbf{x}}_k(t)=\mathbf{V}+\boldsymbol{\omega}\times\mathbf{r}_k(t)
$$

单基地视线方向 $\mathbf{n}_k(t)$ 从雷达到散射点，则：

$$
f_k(t)\approx \frac{2}{\lambda}
\dot{\mathbf{x}}_k(t)\cdot\mathbf{n}_k(t)
$$

展开得到：

$$
f_k(t)\approx
\underbrace{\frac{2}{\lambda}\mathbf{V}\cdot\mathbf{n}_k(t)}_{\text{standard Doppler}}
+
\underbrace{\frac{2}{\lambda}(\boldsymbol{\omega}\times\mathbf{r}_k(t))\cdot\mathbf{n}_k(t)}_{\text{micro-Doppler}}
$$

频谱上：

- standard Doppler 表现为整体频移或中心谱线；
- micro-Doppler 表现为围绕该中心的周期性展宽和边带；
- 机身散射强时，中心水平线明显；
- 叶片散射强时，宽带包络和边带更明显。

在代码中应避免把二者分开生成后非相干相加功率。应在复相位上叠加：

$$
\phi_k(t)=\phi_{k,0}
+2\pi\int_0^t
\left[f_{D,\mathrm{body}}(\xi)+f_{mD,k}(\xi)\right]d\xi
$$

再生成：

$$
s(t)=\sum_k A_k e^{j\phi_k(t)}
$$

## 5. 叶片散射点是否合理

### 5.1 单个叶片尖端散射点

特点：

- 最简单，适合验证正弦 micro-Doppler；
- 峰值频偏最大，因为叶尖速度最大；
- 谱图更像清晰的正弦轨迹或稀疏边带；
- 无法表现整片叶片沿半径方向的速度分布。

适合：第一阶段验证、公式对比、相位连续性调试。

### 5.2 多散射点叶片

把每片叶片离散为多个半径位置：

$$
\mathbf{r}_{m}(t)=\rho_m[\cos(\omega t+\phi),\sin(\omega t+\phi),0]
$$

其中 $0<\rho_m<R$。不同 $\rho_m$ 对应不同峰值频偏：

$$
f_{mD,\max}(\rho_m)=\frac{2}{\lambda}\omega\rho_m
$$

因此多个散射点会把能量从低频到叶尖最大频偏连续填充，MDS 更接近文献中的“填充的正弦包络”。

适合：数据生成主方案，推荐每片叶片 3 到 10 个点。

### 5.3 完整叶片网格

完整网格可以表达叶片形状、遮挡和姿态，但在 Sionna RT 中逐帧旋转完整网格并高采样率求解非常昂贵，而且仍不等于严格全波 RCS。若要使用，建议：

- 用低帧率作为校准，不用于 50 kHz 全量数据；
- 或离线计算/拟合 RCS 随角度的函数，再回填到点散射体权重 $A_k(t)$；
- 不要把“网格更复杂”误认为“MDS 一定更准确”，关键是相干相位和 RCS 标定。

## 6. 采样率和 STFT 参数

### 6.1 Snapshot rate

先估算最大频偏：

$$
f_{\max}=|f_{D,\max}|+\frac{\eta}{\lambda}2\pi R f_{\mathrm{rot}}
$$

其中单基地 $\eta=2$，one-way $\eta=1$。采样率建议：

$$
f_s \ge 2.5 f_{\max}
$$

典型参数：

| 参数 | 3.5 GHz | 28 GHz |
|------|---------|--------|
| $\lambda$ | 85.7 mm | 10.7 mm |
| $R=0.15$ m, $f_{\mathrm{rot}}=100$ Hz 的单基地 $f_{mD,\max}$ | 2.2 kHz | 17.6 kHz |
| 推荐 $f_s$ | 8 到 10 kHz | 50 到 100 kHz |

### 6.2 STFT 窗长

频率分辨率：

$$
\Delta f=\frac{f_s}{N_{\mathrm{win}}}
$$

时间分辨率约为：

$$
\Delta t_{\mathrm{win}}=\frac{N_{\mathrm{win}}}{f_s}
$$

如果 $f_s=50$ kHz：

| `N_win` | 频率分辨率 | 窗时长 | 适用 |
|---------|------------|--------|------|
| 512 | 97.7 Hz | 10.2 ms | 看 100 Hz 边带刚好够，但较粗 |
| 1024 | 48.8 Hz | 20.5 ms | 折中 |
| 2048 | 24.4 Hz | 41.0 ms | 适合展示边带和包络 |
| 4096 | 12.2 Hz | 81.9 ms | 频率细，但时间变化被抹平 |

推荐默认：

```text
fs = 50e3 或 100e3
T_obs >= 0.2 s
N_win = 1024 或 2048
overlap = 75% 到 87.5%
window = Hann
return_onesided = False
fftshift 生成零中心频率轴
```

### 6.3 避免混叠

要同时检查两种混叠：

1. Doppler 频率混叠：$f_s/2$ 必须高于最大正负频偏。
2. 相位差分混叠：若用相邻相位差估计频率，要求每采样相位变化小于 $\pi$，即也等价于 $f_s>2f_{\max}$。

若只是 `np.unwrap(np.angle(signal))`，大 $\beta$ 条件下很容易失败。项目报告中已经出现过这个问题，因此建议保留解析几何相位作为验证基准。

## 7. 如何验证 MDS 正确

推荐按以下阶梯验证，每一步只增加一个复杂度。

| 步骤 | 模型 | 应看指标 |
|------|------|----------|
| 1 | 单个匀速点目标 | 相位线性、Doppler 与 $\eta \mathbf{V}\cdot\mathbf{n}/\lambda$ 一致 |
| 2 | 单个旋转散射点 | 瞬时频率为正弦、峰值频偏等于 $\eta\omega R/\lambda$ |
| 3 | 两个相隔 $\pi$ 的叶尖点 | 周期结构正确、偶次谐波增强、两点相干叠加无异常断裂 |
| 4 | 单旋翼多点叶片 | 谱图由稀疏轨迹变为填充包络，边带间隔等于 $f_{\mathrm{rot}}$ |
| 5 | 四旋翼 | 宽带展宽、机身中心线、旋翼相位和叶片数对应的谐波结构 |
| 6 | 加入机身平动 | 整个谱图中心移动到 $f_D$，micro-Doppler 展宽相对中心保持 |
| 7 | 加入 Sionna 多径 | 主结构不消失，可能出现延迟副本或弱鬼影 |
| 8 | 与论文/解析模型对比 | 峰值频偏、边带间隔、旋转周期、包络形状、功率相对分布 |

建议记录的数值指标：

- 峰值频偏误差：$|\hat f_{\max}-f_{\max}|/f_{\max}$；
- 旋转周期误差：谱图/包络周期是否为 $1/f_{\mathrm{rot}}$ 或与叶片数相关的闪烁周期；
- 边带间隔：频谱峰间距是否为 $f_{\mathrm{rot}}$ 或对称配置下的倍频；
- 相位连续性：`np.diff(np.unwrap(angle))` 是否出现非物理尖峰；
- 能量占比：预期 Doppler 带内能量占总能量比例；
- 多径稳定性：路径数跳变是否导致谱图伪纹。

## 8. 推荐最终数据生成链路

### 8.1 流程图

```text
[1] Blender 建模
    - 地面、墙体、建筑物、障碍物
    - UAV 外观可用于可视化和静态遮挡
    - 每个物体按材料拆分
    - 单位保持 1 unit = 1 m，Z-up

        |
        v

[2] 导出到 Sionna RT
    - 推荐：mitsuba-blender 直接导出 XML
    - 备选：PLY + materials.json -> generate_scene_xml.py -> XML
    - load_scene(xml)

        |
        v

[3] 坐标与链路设置
    - scene.frequency = 3.5e9 或 28e9
    - scene.tx_array / scene.rx_array
    - 添加 BS/Radar: Transmitter 或 Receiver
    - 添加 UAV 参考点: drone_rx 或 uav_tx
    - 保证 Blender UAV 中心 = Sionna UAV 参考点 = modulator body_position

        |
        v

[4] 静态 PathSolver
    - max_depth = 0/3/5 分级验证
    - los/specular/diffuse 按场景打开
    - paths = PathSolver()(scene, ...)
    - a_static, tau_static = paths.cir(..., normalize_delays=False)

        |
        v

[5] UAV 运动模型
    - body_position(t) = body0 + V_body * t
    - rotor_center_j(t) = body_position(t) + arm_offset_j
    - blade_point_jm(t) = rotor_center_j(t) + rho_m [cos(omega t + phi), sin(...), 0]
    - 单基地使用 round-trip phase，双基地使用 Tx/Rx 距离和

        |
        v

[6] 动态 CIR/CFR 或复信号
    - 快速路线：a_dynamic(t) = a_static * modulation_p(t)
    - 高保真改进：每条路径 p 根据 AoD/AoA 计算不同投影 n_p
    - signal(t) = sum_p a_dynamic_p(t)
    - 可选：CFR H[k,t] = sum_p a_p(t) exp(-j2pi f_k tau_p)

        |
        v

[7] STFT/MDS
    - complex STFT, return_onesided=False
    - Hann window
    - fftshift
    - S_dB = 20log10(abs(Z)+eps), max-normalized

        |
        v

[8] 数据保存
    - .npz 保存 signal, f_stft, t_stft, S_dB
    - 保存参数 config.json/yaml
    - 保存 a_static/tau_static 或路径统计
    - 保存标签：UAV 类型、旋翼数、叶片数、R、RPM、V_body、SNR、场景名、频段、单/双基地

        |
        v

[9] 可视化与验证
    - MDS 图
    - Doppler profile
    - 理论峰值频偏线
    - 边带间隔标注
    - 相位连续性诊断
```

### 8.2 保存格式建议

每个样本建议保存：

```text
sample_xxxxxx.npz
  signal_complex: complex64 [N]
  f_stft: float32 [F]
  t_stft: float32 [T]
  S_dB: float32 [F, T]
  a_static: complex64 optional
  tau_static: float32 optional

sample_xxxxxx.json
  scene_name
  carrier_freq
  radar_mode
  tx_position
  rx_position
  body_position0
  body_velocity
  n_rotors
  n_blades_per_rotor
  blade_radius
  blade_points_per_blade
  rotation_freq
  rotor_phases
  body_amplitude
  blade_amplitude
  fs
  obs_duration
  stft_nwin
  stft_overlap
  snr_db
  max_depth
  los/specular/diffuse/diffraction
```

## 9. 最容易出错的 10 个关键点

1. 把 one-way Sionna 通信链路直接当成 monostatic radar，忘记 round-trip 因子 2。
2. 用 pathloss/RSS 做 STFT，而不是用相干复 CIR/CFR/基带信号。
3. `body_position`、Blender UAV 中心、Sionna `drone_rx.position`、modulator 坐标不一致。
4. 采样率太低，28 GHz、R=0.15 m、100 Hz 旋翼至少需要 50 kHz 级别。
5. 相位 unwrap 在大调制指数下失败，却误以为谱图异常来自物理模型。
6. 每条多径都使用同一微多普勒投影，导致复杂场景下路径方向差异被低估。
7. 把叶片尖端单点模型写成“真实叶片散射”，论文表述过度。
8. 没有区分机身 standard Doppler 中心偏移和叶片 micro-Doppler 展宽。
9. 没有保存完整参数和随机相位，数据集不可复现。
10. 逐帧 PathSolver 路线在高采样率下计算量爆炸，适合验证，不适合大规模数据生成。

## 10. 当前项目可复用文件

| 文件 | 可复用内容 |
|------|------------|
| `src/micro_doppler_modulator.py` | UAV 参数类、散射点轨迹、round-trip/双基地距离、CIR 调制、STFT |
| `src/micro_doppler_integration_demo.py` | 静态 RT -> CIR -> micro-Doppler 调制 -> MDS 的集成范例 |
| `src/step3_quadrotor_spectrogram.py` | 四旋翼 8 叶片 MDS 生成和参数设置 |
| `src/step7_dynamic_drone_demo.py` | Mode A 逐步 RT 与 Mode B 快速调制的工程对比 |
| `src/rt_micro_doppler_vs_standard_doppler.py` | 逐 snapshot 更新四旋翼点散射体并重跑 PathSolver 的验证路线 |
| `src/rt_micro_doppler_vs_standard_doppler_single.py` | 单旋翼/双叶片逐帧 RT 验证，已标注 one-way 限制 |
| `src/step8_blender_scene_interface.py` | Blender XML/PLY/程序化场景统一加载接口 |
| `src/step9_pipeline_pathloss_csi.py` | Blender -> RT -> Pathloss/CSI 数据管线，可扩展保存 MDS |
| `tools/blender_to_sionna/generate_scene_xml.py` | PLY + 材料映射生成 Mitsuba/Sionna XML |
| `tools/cir_to_cfr/dataset_CIR_to_CFR.py` | CIR -> OFDM CFR，可用于基于子载波复信道生成 MDS |
| `docs/Blender_Sionna_MicroDoppler_Chain.md` | 现有 Blender + Sionna + micro-Doppler 链路说明 |
| `docs/three_step_validation_report.md` | 标准 Doppler、单散射点、四旋翼谱图的验证论据 |
| `docs/Step5_完成报告.md` | CIR 管道集成验证、性能和三场景对比 |

## 11. 建议的论文/组会表述

推荐表述：

> 我们没有修改 Sionna RT 的射线追踪核心，而是将 Sionna RT 用于静态环境和多径 CIR 建模，并在目标层面引入 UAV 机身及旋翼散射点模型。机身平动通过标准 Doppler 相位项体现，旋翼运动通过散射点相对机身的周期性距离变化产生 micro-Doppler 相位调制。最终对相干复基带序列进行 STFT，生成 UAV MDS。该方法是基于点散射体和远场近似的混合 RT-解析仿真链路，可用于 MDS 数据生成和参数敏感性研究。

避免表述：

> Sionna RT 已经完整仿真了真实 UAV 旋翼的雷达散射。

更严谨的贡献点：

- 给出了 Sionna RT 静态多径与 UAV 旋翼 micro-Doppler 的可复用集成方法；
- 建立了 standard Doppler 与 micro-Doppler 的相干相位统一模型；
- 提供了单点、单旋翼、四旋翼、RT 多径场景的分级验证；
- 形成了可批量生成 MDS 数据集的工程管线。

## 12. 参考资料

### 项目内部资料

- `README.md`
- `docs/Micro_Doppler_Validation_Plan.md`
- `docs/Blender_Sionna_MicroDoppler_Chain.md`
- `docs/three_step_validation_report.md`
- `docs/Step5_完成报告.md`
- `docs/pipeline_report.md`
- `papers/Micro-Doppler_Signature_Simulation_of_Multirotor_UAVs_Using_Ray_Tracing.pdf`
- `papers/Micro-Doppler_Signature-Based_Detection_Classification_and_Localization_of_Small_UAV.pdf`

### Sionna RT 官方资料

- Sionna RT `Paths` 文档：`https://nvlabs.github.io/sionna/rt/api/paths.html`
- Sionna RT `Scene/load_scene` 文档：`https://nvlabs.github.io/sionna/rt/api/scene.html`
- Sionna RT Scene Editing 教程：`https://nvlabs.github.io/sionna/rt/tutorials/Scene-Edit.html`
- Sionna RT Radio Materials 文档：`https://nvlabs.github.io/sionna/rt/api/radio_materials.html`
- Sionna RT Technical Report, Path Solver：`https://nvlabs.github.io/sionna/rt/tech-report/S3.html`
