# HIM for Unitree H1

[HIMLoco](https://github.com/InternRobotics/HIMLoco) adapted to Unitree H1 in Isaac Gym. H1 joint configuration, PD gains and rewards are in [h1_config.py](legged_gym/legged_gym/envs/h1/h1_config.py).

![H1 locomotion in simulation](h1.gif)

[Training plots](plots.png) · [Overview](him_h1.png)

## Setup

Ubuntu 20.04, Python 3.7.16, PyTorch 1.10.0+cu113, Isaac Gym Preview 4. Experiments ran on an RTX 4090 with driver 525.147.05.

```bash
conda create -n himloco python=3.7.16
conda activate himloco
python -m pip install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
```

Download [Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym) and install it:

```bash
python -m pip install -e /path/to/isaacgym/python

git clone https://github.com/dancher00/him-humanoid-h1.git
cd him-humanoid-h1
python -m pip install -e rsl_rl
python -m pip install -e legged_gym
```

Use the `rsl_rl` and `legged_gym` versions in this fork.

## Train

From the repository root:

```bash
cd legged_gym/legged_gym/scripts
python train.py --task=h1
```

## Play and export

From the same directory, after training:

```bash
python play.py --task=h1
```

Checkpoints: `legged_gym/logs/rough_h1`. Exported policies: `legged_gym/logs/rough_h1/exported/policies`.

See the [upstream README](projects/himloco/README.md) for the HIM method, authors and citation.
