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

## Running the Code

### File: `quadruped_jump.py`

This file implements the jumping controller using the optimized parameters.  
It launches the simulation of the different optimized jump types.  
When executed, the program will prompt the user to specify the desired jump type through command-line arguments.

```bash
# Example: start the simulation of a front jump
python quadruped_jump.py Front_jump
```
#### Arguments

Different jump types can be specified as command-line arguments when launching the script.  
The available options are:

- `Front_jump` – Performs a standard forward jump.  
- `Front_jump_s` – Executes forward jump maximising the speed.  
- `Side_jump_L` – Performs a lateral jump to the left.  
- `Side_jump_R` – Performs a lateral jump to the right.  
- `Cw_twist` – Executes clockwise jumps.
- `Ccw_twist` – Executes counterclockwise jumps.

Each option corresponds to a specific optimized jump configuration designed for stability and performance after testing.

### File: `quadruped_jump.py`
This file implements differents optimisations function for each type of jump. When executed, the program will prompt the user to specify the optimisation type through command-line arguments.
```bash
# Example: start the optimisation for all the gains value in the case of a vertical jump 
python quadruped_jump_opt.py opt_stab
```
### Arguments 
Different optimisation can be specified as command-line arguments when launching the script.  
The available options are:
- `opt_stab` – Optimise gains for small verticals jumps.
- `opt_front` – Optimise jump profile parameters for front jumps.
- `opt_side` – Optimise jump profile parameters for a side jump.
- `opt_twist` – Optimise jump profile parameters for a twist jump (inverted for the other side). 
- `opt_speed` – Optimise the speed in the x axis for forwrd jumps.

## Members
LUYET Maxime <br>
KUGATHASAN Ashvin <br>
BAUER Arthur


## References

```
[1] G. Bellegarda, M. Shafiee, M. E. Özberk and A. Ijspeert, "Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 1443-1450.

[2] Akiba, T., et al, "Optuna: A Next-generation Hyperparameter Optimization Framework," in Proceedings of the 25th ACM SIGKDD International Conference on Knowledge Discovery & Data Mining, 2019, pp. 2623–2631.
```
