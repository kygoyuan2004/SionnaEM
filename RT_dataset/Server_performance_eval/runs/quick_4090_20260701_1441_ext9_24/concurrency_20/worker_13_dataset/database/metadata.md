# Sionna RT UAV STFT Dataset Metadata Summary

- Total samples: `1`
- Channel model: `sionna_rt_pathsolver`
- Important: this dataset calls `sionna.rt.PathSolver` for every configured RT snapshot.
- RT snapshot stride: `4`
- RT max depth: `2`

## Class Counts

| class_id | samples | degree | body speed [m/s] |
| --- | ---: | ---: | ---: |
| `level_v0` | 0 | 0.0 | 0.0 |
| `pitch30_v10` | 1 | 30.0 | 10.0 |
| `pitch45_v10` | 0 | 45.0 | 10.0 |
| `single_blade_v0` | 0 | 0.0 | 0.0 |

## First Sample Snapshot

- `sample_id`: `pitch30_v10_0000`
- `class_id`: `pitch30_v10`
- `image_path`: `images/pitch30_v10/pitch30_v10_0000.png`
- `tensor_path`: `tensors/pitch30_v10/pitch30_v10_0000.npz`
- `scene_name`: `floor_wall`
- `scene_channel_model`: `sionna_rt_pathsolver`
- `carrier_frequency_hz`: `28000000000.0`
- `sampling_rate_hz`: `20000.0`
- `num_snapshots`: `256`
- `snapshot_duration_s`: `0.0128`
- `stft_window_size`: `48`
- `stft_overlap`: `36`
- `stft_nfft`: `512`
- `stft_window_duration_s`: `0.0024`
- `stft_frequency_resolution_hz`: `416.6666666666667`
- `rotor_frequency_hz`: `23.61979199198774`
- `rotor_period_s`: `0.042337375381595994`
- `omega_rad_s`: `148.4075300026954`
- `omega_h_rad_s`: `138.108768547849`
- `omega_theta_rad_s`: `148.4075300026954`
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
- `blade_radius_m`: `0.020542162608690576`
- `num_uavs`: `1`
- `num_rotors`: `4`
- `num_blades_per_rotor`: `2`
- `points_per_blade`: `3`
- `num_scatterers`: `25`
- `doppler_body_theory_hz`: `-1867.958740234375`
- `micro_doppler_max_theory_hz`: `493.891357421875`
- `doppler_support_min_theory_hz`: `-2361.810791015625`
- `doppler_support_max_theory_hz`: `-1374.0673828125`
- `stft_support_min_observed_hz`: `-10000.0`
- `stft_support_max_observed_hz`: `9960.9375`
- `stft_peak_observed_hz`: `-1875.0`
- `window_ratio_Twin_over_Trot`: `0.056687500780770575`
- `window_strict_metric`: `0.4221918515212351`
- `window_check_pass`: `True`
- `random_seed`: `20360630`
- `noise_snr_db`: `21.73287531710634`
- `created_time`: `2026-07-01T14:43:57+08:00`
- `initial_body_position_x`: `-0.04513812098392342`
- `initial_body_position_y`: `-0.0004908260507501816`
- `initial_body_position_z`: `2.9912052960779074`
- `initial_rotor_phase_rad`: `3.8612834124250752`
- `bs_position_x`: `-20.0`
- `bs_position_y`: `0.0`
- `bs_position_z`: `3.0`
- `sample_elapsed_s`: `8.294194175978191`
- `scene_channel_model`: `sionna_rt_pathsolver`
- `rt_solver`: `sionna.rt.PathSolver`
- `rt_max_depth`: `2`
- `rt_los`: `True`
- `rt_specular_reflection`: `True`
- `rt_diffuse_reflection`: `False`
- `rt_diffraction`: `False`
- `rt_refraction`: `False`
- `rt_snapshot_stride`: `4`
- `rt_snapshots_solved`: `65`
- `rt_elapsed_s`: `5.250527186086401`
- `rt_total_valid_paths`: `1625`
- `rt_min_valid_paths_per_solved_snapshot`: `25`
- `rt_max_valid_paths_per_solved_snapshot`: `25`
- `rt_cir_sampling_frequency_hz`: `122880000.0`
