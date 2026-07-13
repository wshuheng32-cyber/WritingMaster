# Repository Guidelines

## Project Structure & Module Organization
This repository is a small Python workflow for hand-eye calibration with a RealSense camera and a robotic arm. Top-level scripts are the main entry points:

- `collect_data.py`: capture calibration images and matching robot poses.
- `compute_in_hand.py`: compute eye-in-hand calibration.
- `compute_to_hand.py`: compute eye-to-hand calibration.
- `save_poses.py` and `save_poses2.py`: convert recorded poses into homogeneous matrices.
- `libs/`: shared helpers for logging and file/IP utilities.
- `picture/`: documentation images used by `README.md`.
- `config.yaml`: checkerboard dimensions and square size.

Runtime data is written under `eye_hand_data/dataYYYYMMDD.../` and should not be hand-edited.

## Build, Test, and Development Commands
Use a local virtual environment for development:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Common workflows:

```bash
python3 collect_data.py      # collect images and robot poses
python3 compute_in_hand.py   # solve eye-in-hand calibration
python3 compute_to_hand.py   # solve eye-to-hand calibration
python3 -m py_compile *.py libs/*.py
```

`py_compile` is the fastest local syntax check before opening a PR.

## Coding Style & Naming Conventions
Follow existing Python style: 4-space indentation, `snake_case` for functions and variables, and short module-level scripts. Keep hardware-related constants in `config.yaml` or clearly named locals. Reuse `CommonLog` for logging instead of ad hoc `print()` calls, except for simple one-off script output already established in the repo.

## Testing Guidelines
There is no automated test suite yet. For changes, run `python3 -m py_compile *.py libs/*.py` and validate with a real or recorded calibration dataset. If logic changes affect pose conversion or folder selection, include a reproducible manual check and expected output in the PR description.

## Commit & Pull Request Guidelines
Recent history uses short, task-focused commit messages, often in Chinese, for example `修改获取机械臂状态json协议返回多个字典的情况`. Keep commits narrow and descriptive; one behavior change per commit is preferred.

PRs should include:

- the calibration mode affected (`eye-in-hand` or `eye-to-hand`)
- hardware assumptions or IP/config changes
- commands run for validation
- sample logs or output matrices when behavior changes

## Security & Configuration Tips
Do not commit captured datasets, robot-specific secrets, or environment-specific IP changes without explanation. Review `get_ip()` behavior and `config.yaml` values before running against production hardware.
