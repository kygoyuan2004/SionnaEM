#!/usr/bin/env bash
set -euo pipefail

cd /home/zfh/SionnaEM
source /home/zfh/miniconda3/etc/profile.d/conda.sh
conda activate sionna_rt

python RT_dataset/scripts/build_rt_uav_stft_dataset.py --resume
