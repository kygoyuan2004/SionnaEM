# Sionna RT UAV STFT Dataset

This directory is the PathSolver-based replacement for `/home/zfh/SionnaEM/dataset`.

## What is different

- `dataset/` used a fast LOS geometric point-scatterer approximation.
- `RT_dataset/` calls `sionna.rt.PathSolver` for the configured RT snapshots.
- UAV body/blade scatterers are represented as passive Sionna RT `Receiver` probes.
- For every RT snapshot, probe positions are updated, `PathSolver` is called, and the one-way BS-to-probe CIR is used to form the monostatic return:

```text
s(t) = sum_k w_k h_k(t)^2
```

Here `h_k(t)` is obtained from Sionna RT, not from the previous hand-written free-space formula.

## Configuration

The main configuration is:

```text
configs/rt_dataset_config.yaml
```

It mirrors the original dataset configuration:

- `samples_per_class: 10`
- classes: `level_v0`, `pitch30_v10`, `pitch45_v10`, `single_blade_v0`
- `sampling_rate_hz: 20000`
- `num_snapshots: 2048`
- `stft_window_size: 48`
- `stft_overlap: 36`
- `stft_nfft: 512`

RT-specific fields:

- `scene_channel_model: sionna_rt_pathsolver`
- `rt_max_depth: 2`
- `rt_los: true`
- `rt_specular_reflection: true`
- `rt_snapshot_stride: 1`

`rt_snapshot_stride: 1` means full RT for every snapshot. This is the strictest setting and is very expensive for 3000 samples.

## Generate Dataset

Use the `sionna_rt` conda environment:

```bash
cd /home/zfh/SionnaEM
source /home/zfh/miniconda3/etc/profile.d/conda.sh
conda activate sionna_rt
python RT_dataset/scripts/build_rt_uav_stft_dataset.py --resume
```

Outputs:

- `images/<class_id>/*.png`
- `tensors/<class_id>/*.npz`
- `database/metadata.csv`
- `database/manifest.jsonl`
- `database/metadata.md`
- `database/timing.csv`

## Quick Smoke Test

For a fast correctness test:

```bash
cd /home/zfh/SionnaEM
source /home/zfh/miniconda3/etc/profile.d/conda.sh
conda activate sionna_rt
python RT_dataset/scripts/build_rt_uav_stft_dataset.py \
  --samples-per-class 1 \
  --classes level_v0 \
  --snapshot-override 16 \
  --rt-snapshot-stride 4
```

The smoke test still calls `sionna.rt.PathSolver`, but only solves a few snapshots.

## Runtime Note

With `samples_per_class: 10`, strict generation is:

```text
4 classes * 10 samples/class * 2048 snapshots/sample
= 81,920 PathSolver calls
```

The script writes metadata and timing after every sample, so it can be interrupted and resumed with `--resume`.
