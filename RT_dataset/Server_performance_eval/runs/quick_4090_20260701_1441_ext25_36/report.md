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

- Largest passing concurrency: `30` workers. First failing concurrency: `31` workers.

## Summary Table

| workers | status | elapsed_s | samples | samples/s | peak_mem_MB | peak_util_% | failure |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 25 | PASS | 14.0 | 25 | 1.7818 | 20183 | 98 |  |
| 26 | PASS | 16.1 | 26 | 1.6194 | 21022 | 99 |  |
| 27 | PASS | 16.1 | 27 | 1.6753 | 21817 | 99 |  |
| 28 | PASS | 17.1 | 28 | 1.6411 | 22621 | 99 |  |
| 29 | PASS | 18.1 | 29 | 1.5983 | 21582 | 99 |  |
| 30 | PASS | 19.2 | 30 | 1.5627 | 22631 | 98 |  |
| 31 | FAIL | 18.2 | 23 | 1.2610 | 24091 | 99 | non-zero return code: worker 4 rc=-6, worker 8 rc=-6, worker 9 rc=1, worker 10 rc=-6, worker 12 rc=-6, worker 23 rc=-6, worker 24 rc=1, worker 29 rc=-6 |

## Interpretation

- This benchmark defines the limit as the largest worker count where every worker exits normally and writes all expected samples.
- The result is workload-specific. `pitch30_v10` with 25 scatterers is much heavier than `single_blade_v0` with 1 scatterer.
- For the most faithful production estimate, rerun with `--full-workload`; the quick default is designed to finish faster.
- Each worker writes into its own shard directory, so this benchmark does not test shared CSV write contention.
