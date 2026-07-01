# RT_dataset 多 GPU / 多 CPU 分段生成策略

## 目标

当前 `RT_dataset` 使用 `sionna.rt.PathSolver` 逐 snapshot 生成数据。单进程严格 RT 生成较慢，因为每个样本需要：

```text
num_snapshots = 2048
num_scatterers = 25  # 普通 UAV 类
每个 snapshot 更新 Receiver 位置并调用 PathSolver
```

在 8 张 A100 和 384 CPU 线程的机器上，可以把数据集按类别和样本编号切分成多个互不冲突的任务并行生成，从而显著加速。

## 核心原则

不要让多个进程同时写同一个：

```text
RT_dataset/database/metadata.csv
RT_dataset/database/manifest.jsonl
RT_dataset/database/timing.csv
```

否则容易出现 CSV 行交错、manifest 损坏、断点续跑判断错误。

推荐方式是：

```text
每个 worker 写到独立 shard 目录
最后统一 merge
```

也就是说，不要 8 个进程同时 `--resume` 写 `/home/zfh/SionnaEM/RT_dataset` 主目录。

## 推荐目录结构

建议使用：

```text
RT_dataset/
  shards/
    gpu0_level_v0_0000_0009/
    gpu1_pitch30_v10_0000_0009/
    gpu2_pitch45_v10_0000_0009/
    gpu3_single_blade_v0_0000_0009/
  images/
  tensors/
  database/
```

每个 shard 是一个完整的小型 `RT_dataset` 输出目录，包含自己的：

```text
images/
tensors/
database/metadata.csv
database/manifest.jsonl
database/timing.csv
logs/
```

最后再把各 shard 的样本复制或合并到主 `RT_dataset`。

## GPU 分配策略

机器有 8 张 A100，其中示例状态里 GPU 2 和 GPU 3 显存已接近满载，因此优先使用空闲 GPU：

```text
优先 GPU: 0, 1, 4, 5, 6, 7
谨慎 GPU: 2, 3
```

每个 worker 使用一个 GPU：

```bash
CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py ...
CUDA_VISIBLE_DEVICES=1 python RT_dataset/scripts/build_rt_uav_stft_dataset.py ...
CUDA_VISIBLE_DEVICES=4 python RT_dataset/scripts/build_rt_uav_stft_dataset.py ...
```

这样每个进程只看到一张 GPU，避免多个 Sionna / Dr.Jit / Mitsuba 进程争同一张卡。

## CPU 分配策略

CPU 是双路 EPYC 9654，共 384 线程。建议不要一次开满 384 个线程，因为 RT 任务还会吃内存和调度资源。

推荐：

```text
每个 GPU worker 分配 16 到 32 个 CPU 线程
6 个 worker 大约使用 96 到 192 线程
保留一部分 CPU 给系统、I/O、日志和后处理
```

如果后续发现 CPU 利用率不高，可以逐步增加 worker 数或线程数。

## 小数据集 4 类各 10 张的并行策略

当前目标是：

```text
level_v0         10 张
pitch30_v10      10 张
pitch45_v10      10 张
single_blade_v0  10 张
总计             40 张
```

最简单的切法是按类别分 4 个 worker：

| Worker | GPU | 类别 |
| --- | ---: | --- |
| worker 0 | 0 | `level_v0` |
| worker 1 | 1 | `pitch30_v10` |
| worker 2 | 4 | `pitch45_v10` |
| worker 3 | 5 | `single_blade_v0` |

如果某一类已经生成了一部分，例如 `pitch45_v10` 已经有 3 张，则对应 worker 使用 `--resume`，它会从缺失样本继续。

## 大数据集 4 类各 1000 张的并行策略

如果以后扩展到每类 1000 张，不建议只按类别切，因为每类一个进程仍然太慢。建议按类别 + 编号范围切分。

例如每类 1000 张可以切成 10 个 shard：

```text
0000-0099
0100-0199
0200-0299
...
0900-0999
```

然后把 shard 分配给 6 到 8 张 GPU。每个 worker 完成一个 shard 后继续领下一个 shard。

## 需要的脚本能力

当前生成脚本已经支持：

```bash
--root
--resume
--samples-per-class
--classes
--rt-snapshot-stride
```

为了最干净地支持编号范围并行，建议后续给脚本增加两个参数：

```bash
--start-index
--end-index
```

例如：

```bash
python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/gpu0_pitch45_0000_0099 \
  --samples-per-class 100 \
  --classes pitch45_v10 \
  --start-index 0 \
  --end-index 99
```

如果不加编号范围参数，也可以通过独立 shard + 后处理重命名完成，但不如脚本内支持稳妥。

## 推荐启动示例

以下是策略示例，不要多个 worker 写同一个主目录。

```bash
cd /home/zfh/SionnaEM
source /home/zfh/miniconda3/etc/profile.d/conda.sh
conda activate sionna_rt

CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/gpu0_level_v0 \
  --samples-per-class 10 \
  --classes level_v0 \
  --resume

CUDA_VISIBLE_DEVICES=1 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/gpu1_pitch30_v10 \
  --samples-per-class 10 \
  --classes pitch30_v10 \
  --resume

CUDA_VISIBLE_DEVICES=4 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/gpu4_pitch45_v10 \
  --samples-per-class 10 \
  --classes pitch45_v10 \
  --resume

CUDA_VISIBLE_DEVICES=5 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/gpu5_single_blade_v0 \
  --samples-per-class 10 \
  --classes single_blade_v0 \
  --resume
```

实际后台运行时，可以把每个命令分别写入独立日志：

```text
RT_dataset/logs/gpu0_level_v0.log
RT_dataset/logs/gpu1_pitch30_v10.log
RT_dataset/logs/gpu4_pitch45_v10.log
RT_dataset/logs/gpu5_single_blade_v0.log
```

## 合并策略

每个 shard 完成后，合并步骤包括：

1. 合并图片目录：

```text
shard/images/<class_id>/*.png -> RT_dataset/images/<class_id>/
```

2. 合并 tensor 目录：

```text
shard/tensors/<class_id>/*.npz -> RT_dataset/tensors/<class_id>/
```

3. 合并 CSV：

```text
把各 shard 的 database/metadata.csv 去掉表头后追加
重新按 class_id 和 sample_id 排序
写成 RT_dataset/database/metadata.csv
```

4. 合并 manifest：

```text
把各 shard 的 manifest.jsonl 按 sample_id 排序后写入主 manifest.jsonl
```

5. 合并 timing：

```text
把各 shard 的 timing.csv 合并，按 sample_id 排序
```

## 验证指标

合并后至少检查：

```text
每类图片数量是否正确
每类 tensor 数量是否正确
metadata.csv 每类行数是否正确
manifest.jsonl 行数是否等于样本总数
timing.csv 行数是否等于样本总数 + 表头
每个 metadata 的 image_path 和 tensor_path 是否真实存在
```

对当前 4 类各 10 张，期望：

```text
images/level_v0              10
images/pitch30_v10           10
images/pitch45_v10           10
images/single_blade_v0       10
tensors/level_v0             10
tensors/pitch30_v10          10
tensors/pitch45_v10          10
tensors/single_blade_v0      10
metadata.csv                 40 rows + header
manifest.jsonl               40 lines
timing.csv                   40 rows + header
```

## 风险与注意事项

1. 多进程不要共享同一个输出根目录。

   这是最重要的。共享主目录会导致 metadata 和 manifest 竞争写入。

2. 先避开 GPU 2 和 GPU 3。

   示例 `nvidia-smi` 中它们显存接近满载，可能不是空闲卡。

3. 每个 worker 单独设置 `CUDA_VISIBLE_DEVICES`。

   否则 Sionna / Dr.Jit 可能默认抢同一张 GPU。

4. 小规模先测。

   建议先每个 worker 跑 1 张，确认没有 GPU 初始化、路径、权限、写文件冲突，再放大到完整 shard。

5. `single_blade_v0` 会明显更快。

   它只有 1 个散射点，而普通 UAV 类有 25 个散射点；RT PathSolver 的 receiver 数少很多。

## 推荐结论

对当前 8 GPU / 384 CPU 机器，推荐：

```text
短期：4 个 worker，按类别并行，各写独立 shard，然后 merge。
中期：6 个 worker，避开 GPU 2/3，按类别和编号范围分 shard。
长期：给脚本增加 --start-index / --end-index，并写一个 merge_shards.py 自动合并 metadata、manifest、timing。
```

这样可以充分利用多 GPU，同时避免数据库文件写入冲突。
