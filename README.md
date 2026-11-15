# HIM locomotion for Unitree H1

Adaptation of [HIMLoco](https://github.com/InternRobotics/HIMLoco) to the Unitree H1 humanoid in Isaac Gym. This repository provides the H1 task configuration, training and playback scripts, and simulation demonstrations.

The work focuses on locomotion using joint-encoder and IMU observations. The original Hybrid Internal Model method and its learning infrastructure come from HIMLoco; this fork applies that infrastructure to a humanoid robot.

## Demonstration

![Unitree H1 locomotion in Isaac Gym](h1.gif)

[Configuration illustration](him_h1.png) · [Training plots](plots.png)

These artifacts document simulation experiments. They do not establish hardware deployment or a numerical improvement over a baseline.

## H1 adaptation

| Component | Implementation |
| --- | --- |
| Robot configuration | [h1_config.py](legged_gym/legged_gym/envs/h1/h1_config.py): H1 asset, initial joint pose, PD gains, action scaling, contact settings, and reward weights |
| Task registration | [envs/__init__.py](legged_gym/legged_gym/envs/__init__.py): the `h1` task uses `LeggedRobot` with `H1RoughCfg` and `H1RoughCfgPPO` |
| Robot assets | [H1 model](legged_gym/resources/robots/h1) |
| Learning infrastructure | [rsl_rl](rsl_rl), including the HIM policy and PPO implementation |
| Training and export | [train.py](legged_gym/legged_gym/scripts/train.py) and [play.py](legged_gym/legged_gym/scripts/play.py) |

The H1 configuration specifies 4,096 parallel environments, 10 policy actions, and the `rough_h1` experiment name. Use the versions of `legged_gym` and `rsl_rl` included in this repository.

## Environment

The original experiments used Ubuntu 20.04, Python 3.7.16, PyTorch 1.10.0+cu113, Isaac Gym Preview 4, and an RTX 4090. The recorded host configuration used NVIDIA driver 525.147.05 and CUDA 12.0; the PyTorch package itself uses CUDA 11.3.

This is a historical Isaac Gym environment. Compatibility with newer Python, PyTorch, or Isaac Lab versions has not been established.

## Installation

Create the environment:

```bash
conda create -n himloco python=3.7.16
conda activate himloco
python -m pip install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
```

Download [Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym), then install its Python package from the extracted directory:

```bash
python -m pip install -e /path/to/isaacgym/python
```

Install this fork from its repository root:

```bash
git clone https://github.com/dancher00/him-humanoid-h1.git
cd him-humanoid-h1
python -m pip install -e rsl_rl
python -m pip install -e legged_gym
```

## Train, play, and export

From the repository root:

```bash
cd legged_gym/legged_gym/scripts
python train.py --task=h1
```

From the same scripts directory, play and export a trained policy:

```bash
python play.py --task=h1
```

Playback resumes a checkpoint from `legged_gym/logs/rough_h1`; train a policy or supply a compatible checkpoint first. The playback script enables policy export to `legged_gym/logs/rough_h1/exported/policies`.

## Attribution

HIM is the method introduced by the [upstream HIMLoco authors](https://github.com/InternRobotics/HIMLoco). This repository retains their learning infrastructure and license notices. See the [upstream project documentation](projects/himloco/README.md) for the method and citation.
