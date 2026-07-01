# RT Dataset: STFT Window and Rotor Omega Check

This report records the relationship between the STFT window and rotor speed for `/home/zfh/SionnaEM/RT_dataset`.

## Key Conclusion

The RT dataset generation does consider the STFT-window / rotor-speed relationship. Each sample metadata records:

- `rotor_frequency_hz`
- `omega_rad_s`
- `rotor_period_s`
- `stft_window_duration_s`
- `window_ratio_Twin_over_Trot`
- `window_strict_metric`
- `window_check_pass`

For the first completed full RT sample:

| field | value |
| --- | ---: |
| `sample_id` | `pitch30_v10_0000` |
| `sampling_rate_hz` | `20000.0` |
| `stft_window_size` | `48` |
| `stft_window_duration_s` | `0.0024` |
| `rotor_frequency_hz` | `24.508442021267147` |
| `rotor_period_s` | `0.04080226719969601` |
| `omega_rad_s` | `153.9910828098885` |
| `window_ratio_Twin_over_Trot` | `0.058820260851041146` |
| `window_strict_metric` | `0.445243825620946` |
| `window_check_pass` | `True` |

So the STFT window covers only about `5.88%` of one rotor period.

## Formula

The database uses rotor frequency in Hz:

```text
f_rot = rotor_frequency_hz
omega = 2*pi*f_rot
T_rot = 1/f_rot
T_win = stft_window_size / sampling_rate_hz
window_ratio = T_win / T_rot
```

For the current configuration:

```text
fs = 20000 Hz
window = 48 samples
T_win = 48 / 20000 = 0.0024 s = 2.4 ms
f_rot ~= 25 Hz
T_rot ~= 1 / 25 = 0.04 s = 40 ms
T_win / T_rot ~= 0.06
```

This is short enough to preserve local instantaneous micro-Doppler structure.

## Why the 2000 Hz Concern Does Not Apply Here

If `fs=2000 Hz` and `window=48`, then:

```text
T_win = 48 / 2000 = 0.024 s = 24 ms
T_win / T_rot ~= 24 / 40 = 0.60
```

That would cover about 60% of a rotor period and would smear the sinusoidal ridge.

But the RT dataset configuration is:

```text
sampling_rate_hz: 20000
stft_window_size: 48
```

Therefore the actual window is `2.4 ms`, not `24 ms`.

## Code Locations

- Configuration: `/home/zfh/SionnaEM/RT_dataset/configs/rt_dataset_config.yaml`
- STFT computation: `/home/zfh/SionnaEM/RT_dataset/scripts/build_rt_uav_stft_dataset.py`
- Window check call: `stft_window_metrics(...)`
- Omega conversion: `omega = 2.0 * pi * rotor_frequency`

## Important Interpretation

Even with a correct STFT window, a full Sionna RT signal may not look like a single clean sine because it contains:

- 25 coherent scatterers: body + 4 rotors * 2 blades * 3 radial points
- Sionna RT path amplitudes and phases from `PathSolver`
- coherent summation `sum_k w_k h_k(t)^2`
- body Doppler for moving classes
- multipath/specular reflection depending on the scene and `rt_max_depth`

Therefore, a non-ideal sine does not automatically mean the STFT window is wrong. For a clean sinusoidal reference, use a single-blade or single-scatterer RT configuration.
