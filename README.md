<div align="center" style="line-height: 1;">
  <a href="https://arxiv.org/abs/2504.04259" target="_blank"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2504.04259-B31B1B?logo=arxiv"/></a>
  <a href="https://discord.com/invite/hk9PGKShPK" target="_blank"><img alt="Discord" src="https://img.shields.io/badge/Discord-orcahand-7289da?logo=discord&logoColor=white&color=7289da"/></a>
  <a href="https://x.com/orcahand" target="_blank"><img alt="Twitter Follow" src="https://img.shields.io/twitter/follow/orcahand?style=social"/></a>
  <a href="https://orcahand.com" target="_blank"><img alt="Website" src="https://img.shields.io/badge/Website-orcahand.com-blue?style=flat&logo=google-chrome"/></a>
  <br>
  <a href="https://github.com/orcahand/orca_core" target="_blank"><img alt="GitHub stars" src="https://img.shields.io/github/stars/orcahand/orca_core?style=social"/></a>
  <a href="https://github.com/orcahand/orca_core/actions/workflows/test.yml" target="_blank"><img alt="Tests" src="https://github.com/orcahand/orca_core/actions/workflows/test.yml/badge.svg"/></a>
</div>

`orca_core` is the local ORBIT control package set for ORCA hands + HELIOS head.

It now uses folder-aligned namespace packages with explicit imports:
- ORCA hand runtime/core modules: `orca_core.*`
- HELIOS head runtime/core modules: `helios_core.*`
- ORCA hand CLI wrappers: `orca_core.scripts.*`
- Shared hardware drivers: `hardware.*`

## Installation
`orca_core` is not installed as a standalone dependency set in this repo. Use the
parent `ORBIT_Teleop` environment files from the repo root; they install this
checkout editable only where the runtime needs it:

```sh
conda env create -f configs/environments/environment_orbit.yml
conda env create -f configs/environments/environment_control.yml
conda env create -f configs/environments/environment_vision.yml
```

- `orbit`: offboard operator, retargeting/model metadata, dev/API checks.
- `control`: Jetson ORCA hand + HELIOS head hardware deps.
- `vision`: Jetson camera runtime only; no ORCA hardware deps.

## Import Policy
Always import from the defining module. Do not import through package roots.

Good:

```python
from orca_core.hand_runtime import OrcaHand
from helios_core.head_runtime import HeliosHeadRuntime
from hardware.head_hardware import HeliosHeadHardware
```

Bad:

```python
from package_root import RuntimeClass
from package_root.utils import helper_fn
```

## Model Paths
- Right hand: `models/orca_hand_right`
- Left hand: `models/orca_hand_left`
- HELIOS head config: `models/helios_head/config.yaml`
- Calibration files: `models/*/calibration.yaml` are device-local and
  gitignored. Run the hand or head calibration flow on each machine before
  runtime; generated calibration data should not be committed.

## ORCA Hand Quick Start
Run hand tools using explicit module paths:

```sh
python -m orca_core.scripts.hand_tension --role helios_upper_right
python -m orca_core.scripts.hand_calibrate --role helios_upper_right
python -m orca_core.scripts.hand_neutral --role helios_upper_right
```

Minimal runtime example:

```python
import time
from orca_core.hand_runtime import OrcaHand

hand = OrcaHand("models/orca_hand_right")
ok, msg = hand.connect()
if not ok:
    raise RuntimeError(msg)

hand.enable_torque()
hand.set_joint_pos({"index_mcp": 90, "middle_pip": 30}, num_steps=25, step_size=0.001)
time.sleep(2.0)
hand.disable_torque()
hand.disconnect()
```

## Tests
From this directory (`orca_core/`):

```sh
python -m pytest -q orca_core/tests helios_core/tests
```
