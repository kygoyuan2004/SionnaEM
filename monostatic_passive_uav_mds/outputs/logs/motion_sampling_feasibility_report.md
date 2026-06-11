# UAV机体与旋翼运动后再次采样生成高精度MDS信号的可行性报告

## 结论

**可行。** 当前仿真采用逐快拍更新几何位置并重新调用 Sionna RT 的方式，在每个采样间隔内同时包含 UAV 机体平移和旋翼转动。STFT 主峰、频带支撑、相位连续性和 Nyquist 条件均满足高精度 micro-Doppler signature (MDS) 生成要求。

## 验证对象

- 单站被动散射模型：基站/雷达 `bs_radar` 是唯一有源发射端，UAV 机体和旋翼采样点作为被动 probe scatterer。
- 采样机制：第 `k` 个快拍在 `t_k` 更新机体位置和旋翼相位；第 `k+1` 个快拍在 `t_k + Δt` 再次更新后重新采样。
- 回波近似：对每个散射点使用一程信道相干和 `h_k(t)^2` 表示单站往返传播相位与损耗。

## 关键参数

| 项目 | 数值 |
| --- | ---: |
| 载频 | 28.000 GHz |
| 波长 | 10.707 mm |
| 快拍数 | 1024 |
| 采样率 | 2000.0 Hz |
| 采样间隔 Δt | 0.500 ms |
| UAV 机体速度 | 1.000 m/s |
| 旋翼频率 | 20.000 Hz |
| 叶片半径 | 0.0200 m |
| 每叶片采样点 | 3 |
| STFT窗长/频率分辨率 | 256 / 7.8125 Hz |
| Nyquist频率 | +/-1000.00 Hz |

## 相邻两次采样之间的运动量

| 指标 | 数值 | 波长归一化 |
| --- | ---: | ---: |
| 机体每采样位移，平均 | 0.5000 mm | 0.0467 λ |
| 机体每采样位移，最大 | 0.5000 mm | 0.0467 λ |
| 旋翼中心每采样位移，最大 | 0.5000 mm | 0.0467 λ |
| 叶片采样点相对旋翼中心位移，最大 | 1.2564 mm | 0.1173 λ |
| 叶片采样点全局位移，最大 | 1.7563 mm | 0.1640 λ |

- 每次采样旋翼角增量为 `3.6000 deg`。
- 叶尖相邻采样理论弦长为 `1.2564 mm`，与轨迹统计的最大相对位移一致。
- 旋翼中心位移与机体位移相同，说明旋翼整体随 UAV 机体同步平移；叶片点还叠加了旋转运动。

## MDS/STFT结果

| 指标 | 理论/判据 | 实测 | 结论 |
| --- | ---: | ---: | --- |
| 机体Doppler主峰 | -186.796 Hz | -187.500 Hz | 通过 |
| MDS频带下界 | -656.264 Hz | -671.875 Hz | 通过 |
| MDS频带上界 | 282.672 Hz | 296.875 Hz | 通过 |
| 最大理论瞬时Doppler绝对值 | < 1000.000 Hz | 656.262 Hz | 通过 |
| 清洁信号最大相邻相位跳变 | < pi rad | 0.760 rad | 通过 |
| 清洁信号瞬时频率最大绝对值 | < 1000.000 Hz | 229.582 Hz | 通过 |
| 估计加噪SNR | 配置 30.0 dB | 29.89 dB | 通过 |

## 判据说明

- STFT频率分辨率为 `7.8125 Hz`，实测主峰与理论机体Doppler误差为 `0.7041 Hz`，小于一个频率bin。
- 实测MDS支撑为 `[-671.875, 296.875] Hz`，理论支撑为 `[-656.264, 282.672] Hz`；边界误差在约两个 STFT bin 内。
- 理论支撑最大绝对频率 `656.264 Hz` 小于 Nyquist `1000.000 Hz`，因此当前采样率下无 Doppler 混叠。
- 清洁回波相邻采样最大相位跳变 `0.760 rad` 小于 `pi`，说明逐采样运动后的相位轨迹可连续展开。
- 平均 path count 为 `1.00`；当前 `max_depth=0`，结果主要验证 LOS 被动散射的一致性。

## 输出文件

- 数据：`/home/zfh/SionnaEM/monostatic_passive_uav_mds/outputs/data/monostatic_passive_uav_mds.npz`
- STFT图：`/home/zfh/SionnaEM/monostatic_passive_uav_mds/outputs/figures/monostatic_passive_uav_mds.png`
- 原始摘要：`/home/zfh/SionnaEM/monostatic_passive_uav_mds/outputs/logs/monostatic_passive_uav_mds_summary.txt`
- 自动验证：`/home/zfh/SionnaEM/monostatic_passive_uav_mds/outputs/logs/validation_report.txt`

## 总体判断

该实验支持“UAV 机体先运动一小段距离，同时旋翼按角速度继续运动一小段弧长，然后再进行下一次采样”的建模方式。只要采样率满足最大机体Doppler与旋翼micro-Doppler之和不超过 Nyquist，并且相邻采样相位跳变可连续展开，STFT 可以从这些逐快拍回波中恢复稳定、高分辨率的MDS结构。当前参数下上述条件全部满足。
