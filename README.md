# MP1: Quadruped Jumping

This repository contains code to implement and optimize a quadruped jumping controller in simulation.
The controller is inspired from the Quadruped-Frog paper [1].

## Quick start 

```bash
# 1. Create environment (with conda, recommended)
conda create -n quadruped python=3.9
conda activate quadruped

# 2. Install dependencies
pip install -r requirements.txt
# If pybullet fails, install through conda before installing requirements again:
conda install conda-forge::pybullet

# 3. Run your first controller script
python quadruped_jump.py
```

## Repository structure

```bash
.
├── env/                     # Library code (DO NOT MODIFY)
│   ├── utils.py             # Filepath utilities
│   └── simulation.py        # Quadruped simulation class (read to see available functions)
├── quadruped_jump.py        # Main script: implement your jumping controller here
├── quadruped_jump_opt.py    # Optimization script: optimize your controller with Optuna
├── profiles.py              # Define force profiles for your controller
└── requirements.txt         # Dependencies
```

## Installation

### 1. Create a virtual environment

We recommend using **conda** (preferred) or **virtualenv** with **Python 3.9** or higher.

```bash
# With conda (recommended)
conda create -n quadruped python=3.9

# With virtualenv (alternative)
python -m venv venv
```

Then, activate your environment every time you intend on using it for this project:

```bash
# With conda (recommended)
conda activate quadruped

# With virtualenv (alternative)
# Note that you need to be in the project directory containing the venv/ folder
source venv/bin/activate    # Linux/Mac OS
venv\Scripts\Activate       # Windows
```

### 2. Install dependencies


```bash
pip install -r requirements.txt
```

⚠️ If pybullet [2] fails to install (compilation issues), install through conda and then install the `requirements.txt` again:

```bash
conda install conda-forge::pybullet
```

## Running the code



## Members
LUYET Maxime <br>
KUGATHASAN Ashvin <br>
BAUER Arthur


## References

```
[1] G. Bellegarda, M. Shafiee, M. E. Özberk and A. Ijspeert, "Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 1443-1450.

[2] Akiba, T., et al, "Optuna: A Next-generation Hyperparameter Optimization Framework," in Proceedings of the 25th ACM SIGKDD International Conference on Knowledge Discovery & Data Mining, 2019, pp. 2623–2631.
```
