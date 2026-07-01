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

- Tested up to `8` concurrent workers and all passed. The true upper bound is at least `8` for this workload.

## Summary Table

| workers | status | elapsed_s | samples | samples/s | peak_mem_MB | peak_util_% | failure |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | PASS | 10.0 | 1 | 0.1000 | 825 | 1 |  |
| 2 | PASS | 3.0 | 2 | 0.6664 | 1633 | 45 |  |
| 3 | PASS | 3.0 | 3 | 0.9995 | 2440 | 89 |  |
| 4 | PASS | 4.0 | 4 | 0.9995 | 3247 | 97 |  |
| 5 | PASS | 4.0 | 5 | 1.2492 | 4046 | 99 |  |
| 6 | PASS | 4.0 | 6 | 1.4990 | 4862 | 99 |  |
| 7 | PASS | 5.0 | 7 | 1.3992 | 5667 | 99 |  |
| 8 | PASS | 5.0 | 8 | 1.5991 | 6474 | 99 |  |

## Interpretation

- This benchmark defines the limit as the largest worker count where every worker exits normally and writes all expected samples.
- The result is workload-specific. `pitch30_v10` with 25 scatterers is much heavier than `single_blade_v0` with 1 scatterer.
- For the most faithful production estimate, rerun with `--full-workload`; the quick default is designed to finish faster.
- Each worker writes into its own shard directory, so this benchmark does not test shared CSV write contention.
