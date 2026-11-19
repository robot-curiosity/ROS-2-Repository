# Curiosity local vision handoff

**Local execution owner:** Codex desktop, Curiosity local training/evaluation chat. Coordinate model jobs here. The cloud Curiosity vision development task should reference this handoff and leave training and evaluation launches to the local owner. Datasets, weights, prediction caches and raw logs stay outside Git.

## Data and holdout

The Windows project is under `%USERPROFILE%\OneDrive\Documents\cleaning-robot-model`. The combined `datasets/trashdet-v1` contains 16,088 train, 2,630 validation and 2,628 nominal test images from official TACO, TACO crowdsourcing, RoLID-11K, MJU-Waste and Open Images backgrounds. The crowdsourced TACO extension appears only in training. The E0 variant, `datasets/trashdet-v1-tacotrain`, is the exact manifest filter produced by `scripts/make_subset.py --train-sources taco`. It retains only the 1,051 official TACO training rows and carries over the full mixed validation and test splits. Official TACO contributes 1,051 train, 224 validation and 224 test images to the full dataset.

Every listed image was read and independently SHA-256 hashed on disk (21,346 full, 6,309 subset). Both inventories matched their manifests. Each dataset had zero cross-split byte-identical pairs and zero cross-split matches by stored source SHA-256, manifest cluster or scene-group ID. Comparing stored 64-bit perceptual hashes at Hamming distance ≤6 yielded the following candidate **pairs**.

| Dataset | Train / validation | Train / test | Validation / test |
|---|---:|---:|---:|
| Full | 8,724 | 8,860 | 1,557 |
| TACO-train | 0 | 0 | 1,557 |

All perceptual matches were within RoLID or within MJU. The builder uses clip and capture-group IDs for those sources and exempts within-source pHash links there. Similar backgrounds can produce matches. Different clip IDs can also share scenery, so these counts leave scene independence unresolved. No test images were rendered during this audit.

Historical run configs and saved metrics point to validation for early stopping, thresholds and model selection. Eleven existing nominal test samples were rendered in the project, and the older TACO workspace also documented test visualizations. Available records contain no quantitative test evaluation or evidence tying the renders to a tuning decision. Keep `test` out of future training, threshold selection and model selection.

## Existing evidence

All four verified configurations used the same 2,630-image validation set including 300 background negatives. Operating points were selected on validation.

| Weights / input | mAP50 | mAP50–95 | AP small | F1 | Background alarms |
|---|---:|---:|---:|---:|---:|
| E1 / 640 | 63.37% | 30.73% | 23.31% | 66.19% | 12/300 |
| E5 / 640 | 63.80% | 30.78% | 23.25% | 66.62% | 8/300 |
| E1 / 960 | 66.77% | 33.19% | 26.78% | 66.98% | 11/300 |
| E5 / 960 | 70.73% | 35.70% | 29.27% | 70.49% | 9/300 |

At 640 the checkpoints differ by 0.05 points of mAP50–95. At 960 E5 leads E1 by 2.51 points. Raising E5's inference size from 640 to 960 adds 4.92 points. Comparing E5/960 with E1/640 gives +4.30 points of F1 at their selected thresholds (paired group bootstrap 95% interval +2.73 to +5.77). The 960 NCNN export scored 35.703% mAP50–95 on the same laptop validation corpus. These comparisons support retaining E5/960 as the reference candidate. Pi and independent robot-camera performance remain unmeasured here.

## E6 and one bounded experiment

E6 is an eight-epoch mosaic-on control warm-started from E5 best. Effective args include 960 px, batch 16, seed 0, deterministic mode, MuSGD, lr0 0.001, lrf 0.1, cosine schedule, one warmup epoch, mosaic 1, close_mosaic 0, scale 0.5, mixup and copy-paste 0, AMP on and cache off. The initial workers=8 attempt failed before training in Windows multiprocessing pipe creation (_winapi.CreateFile, WinError 5). With workers=0, E6 completed eight epochs and trainer-side best-checkpoint validation (2,630 images, mAP50–95 about 35.0%). Both final checkpoints had optimizer state stripped. The wrapper's subsequent standalone model.val call omitted workers, so it could fall back to the installed default of eight. Its run record captures the access denial, and the log shows the partially initialized DataLoader cleanup error. This is consistent with the earlier pipe failure.

The selected experiment recovers E6/960 on the existing verified protocol, with an eight-validation-image CUDA smoke run before a batch-eight pass over all 2,630 validation images. Use E5/960 cached predictions as the reference, with mAP50–95, AP by size, and recall, F1 and background alarms at the frozen E5 confidence of 0.30. This measures the saved control without retraining and gives a like-for-like descriptive comparison. Defer the mosaic-off training arm while scene dependence is unresolved.

Host verified on 2026-11-18 using native Windows Python 3.12.1, PyTorch 2.11.0+cu128 (CUDA build 12.8) and Ultralytics 8.4.95. NVIDIA RTX 5070 Laptop GPU, compute capability 12.0, driver 616.87, 8,151 MiB total VRAM and 5,780 MiB free in a preflight snapshot. The job used a dedicated venv with preinstalled packages, isolated settings and outputs in the Windows Codex local cache under `CuriosityVision/2026-11-18-e6-audit`.

### Recovered result

The eight-image smoke check and the full validation pass succeeded. Inference loaded the existing E6 best checkpoint. The model hash remained unchanged. The validation data fingerprint and evaluator settings matched the cached E5/960 run.

| 960 px weights | mAP50–95 | AP small / medium / large | Recall at 0.30 | F1 at 0.30 | Background alarms at 0.30 |
|---|---:|---:|---:|---:|---:|
| E5 best | 35.70% | 29.27% / 57.90% / 66.68% | 65.94% | 70.49% | 9/300 |
| E6 best | 34.73% | 28.41% / 56.59% / 66.25% | 63.57% | 68.79% | 10/300 |

E6 trails by 0.97 points of mAP50–95 and 1.69 points of F1 at 0.30. These are descriptive validation results from one continuation. E5/960 remains the reference. The E6 checkpoint is usable for inference or a fresh warm start. Exact optimizer-state resume would require an unstripped training checkpoint.

The new output directory contains `smoke.log`, `validation.log`, and `validation/{report.json,predictions.json,threshold_curve.csv}`. The prediction cache can rerun report generation without GPU inference. E6 report SHA-256 is `3688a0087a2fcb98cd997586092c01d931419d76cd3b22e8ebbeb131fbe9d694`. Prediction cache SHA-256 is `89adcbd0a7aaa11ce563ac640e970244a66d7d74b9aa432677aefab0707a082c`.

## Local artifact identities

| Artifact relative to the model project | SHA-256 |
|---|---|
| `datasets/trashdet-v1/manifest.csv` | `eff1201afe96c90009dc50f348980d5af583cded289ba2740ca31f212860bad0` |
| `datasets/trashdet-v1-tacotrain/manifest.csv` | `a1821152e782b1f63c2b40d373fc41c8716ce67593311b859db5756fde11fc41` |
| `runs/e1_full_640/weights/best.pt` | `eb16420057b4042249c524f90e4b20dd9e3748a9959c5ded9e5a2aa679243c32` |
| `runs/e5_full_960/weights/best.pt` | `9fd6ff54cb5003fb39d0bf96c8c042c5e73fd9e1bea24ece9aec480f6ff5e89c` |
| `runs/e6_mosaic1_finetune_w0/weights/best.pt` | `6e6e10a03fde382cbb2879fad8a9b8807dd84f2dcf4adea1bf37338cef2c8384` |
| `runs/e6_mosaic1_finetune_w0/weights/last.pt` | `463492011e4f416a830071ec15f3243e8c6b45e2cc57c2e380227e8b903e91be` |
| E5 NCNN export `model.ncnn.bin` | `b4522ed758a64751d34625a728ef4328ef35ca4cecff6a6bc314a70a74c899e7` |
| E5 NCNN export `model.ncnn.param` | `73bd16d039ae502967c7767d1a7dfa529c12bbe19fe0369d170b8c07586c5683` |
| `reports/verified/comparison_resolution/comparison.json` | `73cf9b089bfebff52399a3bd7aec75d830e7d2c92f797fe874c8230ea3eeefe7` |

The E5 NCNN export is in `weights/verified_e5_960/best_ncnn_model`.
