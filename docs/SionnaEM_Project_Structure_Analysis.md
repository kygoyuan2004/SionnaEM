# SionnaEM — Project Structure Analysis

**Date:** 2026-05-24

---

## Project Overview

SionnaEM is a Micro-Doppler simulation project built on top of **Sionna RT** (ray tracing). The goal is to validate that rotation-induced Micro-Doppler effects can be integrated into Sionna RT for UAV detection/classification simulation.

---

## Top-level Directory Structure

```
SionnaEM/
├── README.md              # Well-written, describes the project
├── requirements.txt        # Python dependencies
├── .gitignore
│
├── src/                    # ★ Main source code (~10 Python scripts)
├── docs/                   # Documents, reports, PDFs (mixed CN/EN)
├── figures/                # Generated figure PNGs (~16)
├── papers/                 # Reference PDFs (3 files)
├── ppt_assets_uav/         # Assets for a PowerPoint/presentation
├── tools/                  # Utility scripts (3 subdirectories)
├── RT_tutorial/            # ★ Embedded copy of sionna-rt tutorial repo
├── sionna-large-radio-maps/# ★ Embedded copy of another repo
└── .claude/                # Claude Code configuration
```

---

## Detailed Directory Breakdown

### `src/` — Main Source Code (~10 Python files)

| File | Purpose |
|------|---------|
| `micro_doppler_modulator.py` | Core MicroDopplerModulator module |
| `micro_doppler_integration_demo.py` | Sionna RT integration demo (Step 5) |
| `verify_modulator.py` | 26-item systematic validation |
| `verify_sionna_rt_env.py` | Environment verification tool |
| `step1_baseline_doppler.py` | Step 1: Standard Doppler baseline |
| `step2_single_scatterer.py` | Step 2: Single scatterer Micro-Doppler |
| `step3_quadrotor_spectrogram.py` | Step 3: Quadrotor UAV spectrogram |
| `step6_static_drone_scene.py` | **New (untracked):** Static drone scene |
| `step7_dynamic_drone_demo.py` | **New (untracked):** Dynamic drone demo |
| `paper_fig2_style_mds.py` | **New (untracked):** Paper Figure 2 style MDS |

### `docs/` — Documentation (mixed Chinese/English)

| Document | Language |
|----------|----------|
| `调研问题与思考框架.md` | Chinese |
| `后续三步计划_第四至六步.md` | Chinese |
| `Micro_Doppler_Validation_Plan.md` | English |
| `Micro_Doppler_Sionna_RT_三步验证报告.md/.pdf` | Chinese |
| `Step4_完成报告.md/.pdf` | Chinese |
| `Step5_完成报告.md/.pdf` | Chinese |
| `BS_Parameter_Reference.md/.pdf` | English |
| `Collaboration_Platform_Research.md/.pdf` | English |
| `Technical_QA.md/.pdf` | English |
| `Work_Summary.md/.pdf` | English |
| `group_meeting_report.html/.pdf` | Chinese |
| `SionnaEM_项目成果汇报.pdf` | Chinese |
| `SionnaEM_项目问答汇总.pdf` | Chinese |

### `tools/` — Utility Scripts

| Path | Description |
|------|-------------|
| `tools/blender_to_sionna/` | Blender scene → Sionna XML converter (has README) |
| `tools/CIR_to_CFR/` | CIR → CFR transformation utilities (no README) |
| `tools/scene_create/` | Scene dataset generation scripts (no README, typo in filename) |
| `tools/md_to_pdf.py` | Loose script at tools root (should be in a subfolder) |

### `figures/` — Generated Figures (16 PNGs, flat)

All 16 PNGs stored in a single flat directory with long descriptive names. No subfolder organization by step or topic.

### `papers/` — Reference Papers (3 PDFs)

- `Micro-Doppler_Signature-Based_Detection_Classification_and_Localization_of_Small_UAV.pdf`
- `Micro-Doppler_Signature_Simulation_of_Multirotor_UAVs_Using_Ray_Tracing.pdf`
- `Micro_Doppler_Sionna_RT_调研报告.pdf`

### `ppt_assets_uav/` — Presentation Collateral

Contains PNG images, an HTML explanation file, an asset manifest, and a PDF — all for a slide deck presentation. These are not source code artifacts.

---

## Identified Issues (6 problems)

### 1. Embedded third-party repos as raw subdirectories

`RT_tutorial/sionna-rt/` and `sionna-large-radio-maps/` are full copies of external Sionna repositories, each with their own `.git` directory. They should be **git submodules** (or added to `.gitignore` if they are only local references). As raw copies, they bloat the repository and have no clear dependency relationship with the project source code.

### 2. Step numbering is broken / README is stale

The README describes Steps 1-6, with Step 6 listed as "待开展" (pending). However, the `src/` directory already contains:

- `step6_static_drone_scene.py` (untracked, new Step 6)
- `step7_dynamic_drone_demo.py` (untracked, new Step 7)
- `paper_fig2_style_mds.py` (untracked, purpose unclear)

The README is out of sync with the actual codebase.

### 3. Mixed-language documentation with duplicates

`docs/` contains a mix of Chinese-named and English-named files. Many exist as `.md` + `.pdf` pairs, creating clutter (8 doc pairs + standalone files). There is no clear separation between working documents and final deliverables.

### 4. Presentation collateral in the repository

`ppt_assets_uav/` contains PNGs, HTML, and PDFs for a slide deck. Several figures are duplicated from `figures/` at different resolutions. This is presentation material, not source code, and does not belong in the repository.

### 5. Underdeveloped and inconsistent `tools/` directory

- `blender_to_sionna/` is well-documented with a README
- `CIR_to_CFR/` has two scripts but no README or usage instructions
- `scene_create/` has a typo in a filename (`sence_` instead of `scene_`) and no README
- `tools/md_to_pdf.py` sits loose at the tools root instead of in its own subfolder

### 6. Flat and unorganized `figures/` directory

All 16 PNGs are in one directory with long descriptive filenames. There are no subfolders by step or topic. Additionally, `ppt_assets_uav/` duplicates several of these figures at different resolutions.

---

## Recommendations

| Priority | Action |
|----------|--------|
| **High** | Convert `RT_tutorial/sionna-rt/` and `sionna-large-radio-maps/` to git submodules, or add them to `.gitignore` if they are local-only references |
| **High** | Update README to reflect Steps 6 & 7, and document `paper_fig2_style_mds.py` |
| **Medium** | Remove `ppt_assets_uav/` from the repo (presentation collateral, not source) |
| **Medium** | Organize `figures/` into subdirectories: `figures/step1/`, `figures/step2/`, etc. |
| **Low** | Fix filename typo: `sence_dataset_create` → `scene_dataset_create` |
| **Low** | Decide on one documentation language or separate into `docs/zh/` and `docs/en/` |
| **Low** | Move `tools/md_to_pdf.py` into its own subdirectory |

---

## Git Status (as of 2026-05-24)

**Modified (staged):**
- `src/step3_quadrotor_spectrogram.py`

**Untracked (new files):**
- `docs/BS_Parameter_Reference.md/.pdf`
- `docs/Collaboration_Platform_Research.md/.pdf`
- `docs/Technical_QA.md/.pdf`
- `docs/Work_Summary.md/.pdf`
- `figures/paper_fig2_style_microhelicopter_vs_quadcopter.png`
- `ppt_assets_uav/` (entire directory)
- `src/paper_fig2_style_mds.py`
- `src/step6_static_drone_scene.py`
- `src/step7_dynamic_drone_demo.py`
- `tools/blender_to_sionna/` (entire directory)
- `tools/md_to_pdf.py`
