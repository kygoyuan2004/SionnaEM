# RT_dataset 同类数据并行生成方案探究

## 结论先行

同一类数据可以并行生成，但不能让多个进程同时写：

```text
/home/zfh/SionnaEM/RT_dataset/database/metadata.csv
/home/zfh/SionnaEM/RT_dataset/database/manifest.jsonl
/home/zfh/SionnaEM/RT_dataset/database/timing.csv
```

最稳妥的方式是：

```text
每个 worker 写独立 shard 目录
每个 worker 负责互不重叠的 sample_index 区间
最后统一合并 images、tensors、metadata、manifest、timing
```

也就是说，推荐采用“一个进程负责 `0000-0099`，另一个负责 `0100-0199`”这种编号区间切分。

当前 `build_rt_uav_stft_dataset.py` 已经支持独立 `--root`，所以独立 shard 是可行的；但它目前还没有 `--start-index / --end-index`，所以完整的区间并行还需要给生成脚本补两个参数，或者写一个外层 wrapper 做重编号。更推荐前者。

## 当前脚本能力

当前生成脚本：

```text
/home/zfh/SionnaEM/RT_dataset/scripts/build_rt_uav_stft_dataset.py
```

已经支持：

```bash
--root
--config
--samples-per-class
--classes
--snapshot-override
--rt-snapshot-stride
--resume
```

其中最关键的是 `--root`。它允许每个 worker 写到自己的目录，例如：

```text
RT_dataset/shards/pitch30_v10_0000_0099/
RT_dataset/shards/pitch30_v10_0100_0199/
RT_dataset/shards/pitch30_v10_0200_0299/
```

这样每个 shard 都有自己的：

```text
images/
tensors/
database/metadata.csv
database/manifest.jsonl
database/timing.csv
```

不会互相抢同一个 CSV。

## 为什么不能直接多个进程生成同一类

假设同时启动两个进程：

```bash
CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/a \
  --samples-per-class 100 \
  --classes pitch30_v10

CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/b \
  --samples-per-class 100 \
  --classes pitch30_v10
```

虽然两个进程写入不同目录，不会损坏 CSV，但它们都会生成：

```text
pitch30_v10_0000
pitch30_v10_0001
...
pitch30_v10_0099
```

这会产生三个冲突：

| 冲突项 | 原因 | 后果 |
| --- | --- | --- |
| `sample_id` 冲突 | 两个 shard 都从 `0000` 开始 | 合并 metadata 时重复 |
| 文件名冲突 | 图片和 tensor 都叫 `pitch30_v10_0000.png/.npz` | 复制到主目录时覆盖 |
| 随机种子冲突 | 当前 seed 由 `class_id + sample_index` 决定 | 两个 shard 可能生成重复样本 |

所以“独立 root”只能解决文件写入互斥问题，不能解决同类样本编号重复问题。

## 方案 A：独立 shard + 后处理重编号

### 可行性

可行，但不推荐作为长期方案。

做法是让每个 shard 先从 `0000` 生成，然后合并时统一重命名：

```text
shard_a/pitch30_v10_0000 -> 主数据集 pitch30_v10_0000
shard_b/pitch30_v10_0000 -> 主数据集 pitch30_v10_0100
shard_c/pitch30_v10_0000 -> 主数据集 pitch30_v10_0200
```

### 优点

- 不需要立刻改生成脚本。
- 只要每个 worker 写独立 `--root`，不会损坏共享 CSV。
- 可以快速试跑。

### 缺点

- 合并脚本必须同时改：
  - 图片文件名
  - tensor 文件名
  - `metadata.csv` 里的 `sample_id`
  - `image_path`
  - `tensor_path`
  - `manifest.jsonl`
  - `timing.csv`
  - tensor 内部 `metadata_json`
- 如果不额外修改随机种子，不同 shard 的 `pitch30_v10_0000` 可能是重复随机样本。
- 后处理逻辑复杂，容易遗漏 tensor 内嵌 metadata。

### 适合场景

适合临时测试，不适合作为正式 1000 张/类的数据生产方案。

## 方案 B：按 sample_index 区间生成

### 可行性

这是最推荐的正式方案。

核心思想是每个 worker 原生生成不同编号：

```text
worker 0: pitch30_v10_0000 - pitch30_v10_0099
worker 1: pitch30_v10_0100 - pitch30_v10_0199
worker 2: pitch30_v10_0200 - pitch30_v10_0299
...
```

这样从生成阶段就保证：

```text
sample_id 不重复
图片文件名不重复
tensor 文件名不重复
random_seed 不重复
metadata 可直接合并
```

### 需要补充的脚本参数

建议给 `build_rt_uav_stft_dataset.py` 增加：

```bash
--start-index
--end-index
```

或者：

```bash
--index-start
--index-count
```

更直观的是第一种：

```bash
--start-index 100
--end-index 199
```

脚本内部主循环从现在的：

```python
for sample_index in range(int(cfg["samples_per_class"])):
```

改成逻辑等价于：

```python
start_index = args.start_index
end_index = args.end_index
for sample_index in range(start_index, end_index + 1):
```

同时 `total` 需要按区间数量计算：

```python
num_indices = end_index - start_index + 1
total = num_indices * len(args.classes)
```

当前随机种子逻辑：

```python
seed = random_seed + sample_index + 100000 * class_order
```

如果 `sample_index` 是全局唯一编号，则种子自然也唯一，不需要额外改。

### 推荐命令形式

例如生成 `pitch30_v10` 的 1000 张，可以切成 10 个 shard，每个 100 张：

```bash
CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/pitch30_v10_0000_0099 \
  --classes pitch30_v10 \
  --start-index 0 \
  --end-index 99 \
  --resume

CUDA_VISIBLE_DEVICES=0 python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --root RT_dataset/shards/pitch30_v10_0100_0199 \
  --classes pitch30_v10 \
  --start-index 100 \
  --end-index 199 \
  --resume
```

如果一张 4090 上先保守开 4 到 8 个并发，可以这样排：

```text
GPU 0:
  worker 0 -> pitch30_v10_0000_0099
  worker 1 -> pitch30_v10_0100_0199
  worker 2 -> pitch30_v10_0200_0299
  worker 3 -> pitch30_v10_0300_0399
```

如果是 8 张 GPU，则更推荐每张 GPU 先 1 个 worker：

```text
GPU 0 -> pitch30_v10_0000_0099
GPU 1 -> pitch30_v10_0100_0199
GPU 2 -> pitch30_v10_0200_0299
GPU 3 -> pitch30_v10_0300_0399
GPU 4 -> pitch30_v10_0400_0499
GPU 5 -> pitch30_v10_0500_0599
GPU 6 -> pitch30_v10_0600_0699
GPU 7 -> pitch30_v10_0700_0799
```

后续 GPU 空出来后继续跑：

```text
pitch30_v10_0800_0899
pitch30_v10_0900_0999
```

## 同一类并行是否互相矛盾

只要满足下面三个条件，就不矛盾：

```text
1. 每个 worker 写独立 --root
2. 每个 worker 的 sample_index 区间不重叠
3. 最后由单进程合并 shard，不并发写主 database
```

这样同一类 `pitch30_v10` 可以并行生成，不会出现：

```text
同名图片覆盖
同名 tensor 覆盖
metadata 重复 sample_id
manifest 重复 sample_id
timing 行冲突
随机种子重复
```

## 合并逻辑

每个 shard 完成后，合并器应当单进程执行：

```text
1. 收集所有 shard/database/metadata.csv
2. 收集所有 shard/database/manifest.jsonl
3. 收集所有 shard/database/timing.csv
4. 检查 sample_id 是否唯一
5. 检查 image_path 和 tensor_path 是否唯一
6. 复制 images/<class_id>/*.png 到主 images/<class_id>/
7. 复制 tensors/<class_id>/*.npz 到主 tensors/<class_id>/
8. 按 class_id、sample_id 排序写主 metadata.csv
9. 按 sample_id 排序写主 manifest.jsonl
10. 按 sample_id 排序写主 timing.csv
```

合并前必须检查：

```text
metadata.csv 中 sample_id 没有重复
图片目标路径不存在冲突
tensor 目标路径不存在冲突
每个 metadata 的 image_path 和 tensor_path 都能在 shard 内找到
```

如果发现重复，应该停止合并，而不是覆盖。

## 对 4090 的并发建议

已在当前 4090 上做过 quick benchmark：

```text
quick 模式：
  snapshot_override = 256
  rt_snapshot_stride = 4
  class = pitch30_v10
  每 worker 1 个样本

结果：
  30 并发 PASS
  31 并发 FAIL
  失败原因是 GPU OOM
```

这说明 quick 模式下 4090 的显存极限大约在 30 个进程附近。但正式生成使用：

```text
num_snapshots = 2048
rt_snapshot_stride = 1
```

正式任务会更慢，进程长期运行时也更容易遇到显存碎片、I/O 排队、日志膨胀、CPU 调度压力。因此正式生产不建议直接开 30 个。

推荐：

```text
单张 4090：
  起步 4 个 worker
  稳定后试 6 个
  再试 8 个
  不建议一开始超过 8 个正式 worker
```

如果是 8 张 A100：

```text
先每张 GPU 1 个 worker，共 8 个 worker
如果单 worker GPU 利用率长期偏低，再考虑每张 GPU 2 个 worker
```

## 两种方案对比

| 方案 | 是否需要改生成脚本 | 是否推荐正式使用 | 主要风险 |
| --- | --- | --- | --- |
| 独立 shard + 后处理重编号 | 不一定 | 不推荐 | metadata/tensor 内嵌信息容易漏改，随机种子可能重复 |
| sample_index 区间生成 | 需要加 `--start-index/--end-index` | 推荐 | 需要小幅修改生成脚本和合并脚本 |

## 推荐最终工程形态

建议拆成三个脚本：

```text
build_rt_uav_stft_dataset.py
  负责生成单个 shard，支持 --start-index / --end-index

launch_parallel_shards.py
  负责读取任务表，按 GPU 启动多个 shard worker

merge_rt_shards.py
  负责单进程合并 shard，做唯一性检查和文件复制
```

任务表可以是 CSV：

```csv
gpu_id,class_id,start_index,end_index,root
0,pitch30_v10,0,99,RT_dataset/shards/pitch30_v10_0000_0099
0,pitch30_v10,100,199,RT_dataset/shards/pitch30_v10_0100_0199
0,pitch30_v10,200,299,RT_dataset/shards/pitch30_v10_0200_0299
0,pitch30_v10,300,399,RT_dataset/shards/pitch30_v10_0300_0399
```

这种设计的好处是：

```text
生成阶段互不写同一个文件
样本编号天然不重复
随机种子天然不重复
失败后可以按 shard 续跑
合并阶段可验证、可重跑、可审计
```

## 最终建议

如果只是短期试验，可以用“独立 shard + 后处理重编号”。

如果要正式生成完整 RT 数据集，建议直接采用“编号区间 shard”方案，并给当前生成脚本增加：

```bash
--start-index
--end-index
```

这是最干净、最不容易出错、最适合后续扩展到 1000 张/类甚至更多样本的并行逻辑。
