# GPU Task Concurrency Benchmark

## Configuration

- GPU id: `0`
- GPU name: `NVIDIA GeForce RTX 4090`
- Generator: `/home/zfh/SionnaEM/RT_dataset/scripts/build_rt_uav_stft_dataset.py`
- Config: `/home/zfh/SionnaEM/RT_dataset/configs/rt_dataset_config.yaml`
- Classes per worker: `pitch30_v10`
- Samples per class per worker: `1`
- Full workload: `False`
- Snapshot override when not full workload: `256`
- RT snapshot stride when not full workload: `4`
- TF memory growth env enabled: `True`

## Result

- Tested up to `24` concurrent workers and all passed. The true upper bound is at least `24` for this workload.

## Summary Table

| workers | status | elapsed_s | samples | samples/s | peak_mem_MB | peak_util_% | failure |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 9 | PASS | 6.0 | 9 | 1.4990 | 7288 | 99 |  |
| 10 | PASS | 6.0 | 10 | 1.6651 | 8089 | 99 |  |
| 11 | PASS | 6.0 | 11 | 1.8320 | 8898 | 99 |  |
| 12 | PASS | 7.0 | 12 | 1.7129 | 9712 | 99 |  |
| 13 | PASS | 7.0 | 13 | 1.8558 | 10515 | 99 |  |
| 14 | PASS | 8.0 | 14 | 1.7488 | 11318 | 99 |  |
| 15 | PASS | 8.0 | 15 | 1.8736 | 12126 | 99 |  |
| 16 | PASS | 9.0 | 16 | 1.7759 | 12943 | 99 |  |
| 17 | PASS | 9.0 | 17 | 1.8864 | 13756 | 98 |  |
| 18 | PASS | 9.0 | 18 | 1.9979 | 14528 | 99 |  |
| 19 | PASS | 10.0 | 19 | 1.8977 | 15337 | 99 |  |
| 20 | PASS | 11.0 | 20 | 1.8155 | 16176 | 99 |  |
| 21 | PASS | 13.0 | 21 | 1.6136 | 16945 | 99 |  |
| 22 | PASS | 12.0 | 22 | 1.8290 | 17771 | 99 |  |
| 23 | PASS | 13.0 | 23 | 1.7660 | 18598 | 99 |  |
| 24 | PASS | 15.0 | 24 | 1.5969 | 19407 | 98 |  |

## Interpretation

- This benchmark defines the limit as the largest worker count where every worker exits normally and writes all expected samples.
- The result is workload-specific. `pitch30_v10` with 25 scatterers is much heavier than `single_blade_v0` with 1 scatterer.
- For the most faithful production estimate, rerun with `--full-workload`; the quick default is designed to finish faster.
- Each worker writes into its own shard directory, so this benchmark does not test shared CSV write contention.
