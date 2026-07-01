# Sionna RT UAV STFT Dataset Metadata Summary

- Total samples: `40`
- Channel model: `sionna_rt_pathsolver`
- Important: this dataset calls `sionna.rt.PathSolver` for every configured RT snapshot.
- RT snapshot stride: `1`
- RT max depth: `2`

## Class Counts

| class_id | samples | degree | body speed [m/s] |
| --- | ---: | ---: | ---: |
| `level_v0` | 10 | 0.0 | 0.0 |
| `pitch30_v10` | 10 | 30.0 | 10.0 |
| `pitch45_v10` | 10 | 45.0 | 10.0 |
| `single_blade_v0` | 10 | 0.0 | 0.0 |

## First Sample Snapshot

- `sample_id`: `pitch30_v10_0000`
- `class_id`: `pitch30_v10`
- `image_path`: `images/pitch30_v10/pitch30_v10_0000.png`
- `tensor_path`: `tensors/pitch30_v10/pitch30_v10_0000.npz`
- `scene_name`: `floor_wall`
- `scene_channel_model`: `sionna_rt_pathsolver`
- `scene_xml_path`: ``
- `scene_tx_name`: ``
- `scene_source`: ``
- `scene_loaded`: ``
- `scene_load_error`: ``
- `carrier_frequency_hz`: `28000000000.0`
- `sampling_rate_hz`: `20000.0`
- `num_snapshots`: `2048`
- `snapshot_duration_s`: `0.1024`
- `stft_window_size`: `48`
- `stft_overlap`: `36`
- `stft_nfft`: `512`
- `stft_window_duration_s`: `0.0024`
- `stft_frequency_resolution_hz`: `416.6666666666667`
- `rotor_frequency_hz`: `24.508442021267147`
- `rotor_period_s`: `0.04080226719969601`
- `omega_rad_s`: `153.9910828098885`
- `omega_h_rad_s`: `143.30484992127606`
- `omega_theta_rad_s`: `153.9910828098885`
- `omega_theta_over_omega_h`: `1.0745699318235418`
- `degree`: `30.0`
- `degree_rad`: `0.5235987755982988`
- `body_speed_m_s`: `10.0`
- `body_velocity_x`: `10.0`
- `body_velocity_y`: `0.0`
- `body_velocity_z`: `0.0`
- `body_acceleration_x`: `0.0`
- `body_acceleration_y`: `0.0`
- `body_acceleration_z`: `0.0`
- `blade_radius_m`: `0.020101285044504064`
- `num_uavs`: `1`
- `num_rotors`: `4`
- `num_blades_per_rotor`: `2`
- `points_per_blade`: `3`
- `num_scatterers`: `25`
- `doppler_body_theory_hz`: `-1867.9547119140625`
- `micro_doppler_max_theory_hz`: `501.972412109375`
- `doppler_support_min_theory_hz`: `-2369.871826171875`
- `doppler_support_max_theory_hz`: `-1365.9822998046875`
- `stft_support_min_observed_hz`: `-10000.0`
- `stft_support_max_observed_hz`: `9960.9375`
- `stft_peak_observed_hz`: `-1875.0`
- `window_ratio_Twin_over_Trot`: `0.058820260851041146`
- `window_strict_metric`: `0.445243825620946`
- `window_check_pass`: `True`
- `random_seed`: `20260630`
- `noise_snr_db`: `20.038566990786997`
- `created_time`: `2026-06-30T16:16:39+08:00`
- `initial_body_position_x`: `0.021935521693647625`
- `initial_body_position_y`: `0.0008234806433878097`
- `initial_body_position_z`: `2.956647538482307`
- `initial_rotor_phase_rad`: `0.4144940630827998`
- `bs_position_x`: ``
- `bs_position_y`: ``
- `bs_position_z`: ``
- `sample_elapsed_s`: `223.3588315480156`
- `scene_channel_model`: `sionna_rt_pathsolver`
- `rt_solver`: `sionna.rt.PathSolver`
- `rt_max_depth`: `2`
- `rt_los`: `True`
- `rt_specular_reflection`: `True`
- `rt_diffuse_reflection`: `False`
- `rt_diffraction`: `False`
- `rt_refraction`: `False`
- `rt_snapshot_stride`: `1`
- `rt_snapshots_solved`: `2048`
- `rt_elapsed_s`: `223.3588315480156`
- `rt_total_valid_paths`: `51200`
- `rt_min_valid_paths_per_solved_snapshot`: `25`
- `rt_max_valid_paths_per_solved_snapshot`: `25`
- `rt_cir_sampling_frequency_hz`: `122880000.0`
