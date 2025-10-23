# Quadruped Jumping Controller# Quadruped Jumping Controller# 🦿 Quadruped Jumping Controller



![Python](https://img.shields.io/badge/python-3.9+-blue.svg)

![PyBullet](https://img.shields.io/badge/pybullet-3.2.7-green.svg)

![Optuna](https://img.shields.io/badge/optuna-4.5.0-orange.svg)![Python](https://img.shields.io/badge/python-3.9+-blue.svg)<div align="center">



---![PyBullet](https://img.shields.io/badge/pybullet-3.2.7-green.svg)



## 1. Introduction![Optuna](https://img.shields.io/badge/optuna-4.5.0-orange.svg)![Python](https://img.shields.io/badge/python-3.9+-blue.svg)



### Overview![PyBullet](https://img.shields.io/badge/pybullet-3.2.7-green.svg)



This project implements an optimized jumping controller for quadruped robots in simulation. The work is based on the research presented in **"Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping"** by Bellegarda et al. (ICRA 2024) [1].Implementation and optimization of a quadruped jumping controller in PyBullet simulation. This project is inspired by the **Quadruped-Frog** paper [1] and demonstrates various jumping behaviors including forward jumps, lateral movements, and rotational twists.![License](https://img.shields.io/badge/license-MIT-orange.svg)



### Context and Motivation



Legged robots have gained significant attention in robotics research due to their ability to navigate complex terrains. Among various locomotion behaviors, jumping is particularly challenging as it requires:---*Implémentation et optimisation d'un contrôleur de saut pour robot quadrupède en simulation*



- Precise timing and coordination of all four legs

- Optimal force distribution to achieve desired trajectories

- Balance between propulsion and stability## Table of Contents</div>

- Adaptation to different jump types (forward, lateral, rotational)



Traditional approaches often rely on hand-tuned parameters or complex model-based controllers. The Quadruped-Frog paper introduces a novel approach using Central Pattern Generators (CPGs) combined with rapid online optimization, enabling quadrupeds to perform continuous, dynamic jumping behaviors.

- [About](#about)---

### Our Implementation

- [Demo Results](#demo-results)

In this project, we implement and extend the Quadruped-Frog methodology to achieve various jumping behaviors on the Unitree A1 quadruped robot in PyBullet simulation:

- [Installation](#installation)## 📋 Table des matières

- **Forward jumping**: Maximizing horizontal distance traveled

- **Lateral jumping**: Side-to-side movements (left and right)- [Project Structure](#project-structure)

- **Twisting jumps**: Rotational movements (clockwise and counter-clockwise)

- **Speed hopping**: High-frequency jumping for rapid locomotion- [Usage](#usage)- [À propos](#-à-propos)



The controller uses CPG oscillators to generate rhythmic force profiles, which are optimized using Optuna [2], a state-of-the-art Bayesian optimization framework.- [VS Code Extensions](#vs-code-extensions)- [Résultats](#-résultats)



### Technical Approach- [Technical Details](#technical-details)- [Installation](#-installation)



The implementation consists of three main components:- [Dependencies](#dependencies)- [Structure du projet](#-structure-du-projet)



1. **Force Profile Generation** (`profiles.py`): CPG-based oscillators that generate periodic ground reaction forces- [References](#references)- [Utilisation](#-utilisation)

2. **Jumping Controller** (`quadruped_jump.py`): Main controller that applies optimized force profiles to achieve desired jumping behaviors

3. **Optimization Framework** (`quadruped_jump_opt.py`): Optuna-based parameter tuning to maximize performance metrics- [Extensions VS Code recommandées](#-extensions-vs-code-recommandées)



------- [Dépendances](#-dépendances)



## 2. Environment Setup- [Références](#-références)



### Prerequisites## About



Before starting, ensure you have the following installed:---



- **Python 3.9 or higher**This project implements a jumping controller for quadruped robots using Central Pattern Generator (CPG) oscillators to generate periodic force profiles. The controller can perform various types of jumps:

- **Anaconda or Miniconda** (recommended for environment management)

- **Git** (for cloning the repository)## 🎯 À propos



### Step-by-Step Installation- **Forward jumps**: Maximizing horizontal distance



#### 2.1 Clone the Repository- **Lateral jumps**: Left and right side movementsCe projet implémente un contrôleur de saut pour robot quadrupède basé sur l'article **Quadruped-Frog** [1]. Le contrôleur utilise des profils de force optimisés pour réaliser différents types de sauts :



```bash- **Twist jumps**: Clockwise and counter-clockwise rotations

git clone https://github.com/Emixm4/Legged-Robots-Miniproject.git

cd Legged-Robots-Miniproject- **Speed hopping**: High-frequency jumping behavior- 🔵 **Sauts avant** : Maximisation de la distance horizontale

```

- 🔴 **Sauts latéraux** : Déplacements gauche/droite

#### 2.2 Create the Conda Environment

The implementation uses **PyBullet** for physics simulation and **Optuna** for parameter optimization.- 🟢 **Sauts avec rotation** : Rotations horaire et anti-horaire

We strongly recommend using a dedicated conda environment to avoid dependency conflicts.

- 🟡 **Sauts rapides** : Hopping à haute fréquence

```bash

# Create a new conda environment named 'quadruped' with Python 3.9---

conda create -n quadruped python=3.9

Le projet utilise **PyBullet** pour la simulation physique et **Optuna** pour l'optimisation des paramètres du contrôleur.

# Activate the newly created environment

conda activate quadruped## Demo Results

```

---

**Note**: You will need to activate this environment every time you work on the project:

```bashThe following videos demonstrate different jumping behaviors achieved after optimization:

conda activate quadruped

```## 🎬 Résultats



#### 2.3 Install Python Dependencies### Forward Jump



Once the environment is activated, install the required packages:Maximizing distance traveled in the forward direction.Voici les différents comportements de saut obtenus après optimisation :



```bash

pip install -r requirements.txt

```[Download Video](videos/Front_jump.mkv)<table>



**Troubleshooting PyBullet Installation**:  <tr>



If PyBullet fails to install due to compilation errors (common on Windows), use conda to install it first:### Speed Hopping    <td align="center">



```bashHigh-frequency jumping with rapid stance-flight transitions.      <b>🔵 Saut avant</b><br>

# Install PyBullet via conda

conda install conda-forge::pybullet      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Front_jump.mkv" width="300"></video><br>



# Then install remaining dependencies[Download Video](videos/Speed_hop.mkv)      <a href="videos/Front_jump.mkv">📥 Télécharger</a><br>

pip install -r requirements.txt

```      <em>Maximisation de distance</em>



#### 2.4 Verify Installation### Lateral Jumps    </td>



Test that everything is properly installed by running a simple simulation:    <td align="center">



```bash**Left Side Jump**      <b>🟡 Hopping rapide</b><br>

python quadruped_jump.py

```      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Speed_hop.mkv" width="300"></video><br>



If successful, a PyBullet simulation window should open, displaying the Unitree A1 robot performing jumping motions.[Download Video](videos/Side_jump_L.mkv)      <a href="videos/Speed_hop.mkv">📥 Télécharger</a><br>



### Dependencies      <em>Sauts à haute fréquence</em>



The project requires the following Python packages (specified in `requirements.txt`):**Right Side Jump**    </td>



| Package | Version | Purpose |  </tr>

|---------|---------|---------|

| **numpy** | 1.26.4 | Numerical computations and array operations |[Download Video](videos/Side_jump_R.mkv)  <tr>

| **pybullet** | 3.2.7 | Physics simulation engine |

| **PyYAML** | 6.0.2 | YAML configuration file parsing |    <td align="center">

| **optuna** | 4.5.0 | Bayesian optimization framework |

### Twist Jumps      <b>🔴 Saut latéral gauche</b><br>

### Alternative: Using virtualenv

      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Side_jump_L.mkv" width="300"></video><br>

If you prefer not to use conda, you can use Python's built-in `venv`:

**Counter-Clockwise Twist**      <a href="videos/Side_jump_L.mkv">📥 Télécharger</a><br>

```bash

# Create virtual environment      <em>Déplacement latéral</em>

python -m venv venv

[Download Video](videos/Ccw_twist_jump.mkv)    </td>

# Activate the environment

# Windows PowerShell:    <td align="center">

venv\Scripts\Activate.ps1

# Windows CMD:**Clockwise Twist**      <b>🔴 Saut latéral droit</b><br>

venv\Scripts\Activate.bat

# Linux/Mac:      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Side_jump_R.mkv" width="300"></video><br>

source venv/bin/activate

[Download Video](videos/Cw_twist_jump.mkv)      <a href="videos/Side_jump_R.mkv">📥 Télécharger</a><br>

# Install dependencies

pip install -r requirements.txt      <em>Déplacement latéral</em>

```

> **Note**: All videos are in MKV format. Click the download links to view them locally.    </td>

---

  </tr>

## 3. Results

---  <tr>

This section presents the various jumping behaviors achieved through optimization of the CPG-based controller.

    <td align="center">

### 3.1 Forward Jump

## Installation      <b>🟢 Rotation anti-horaire</b><br>

**Objective**: Maximize horizontal distance traveled while maintaining stability.

      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Ccw_twist_jump.mkv" width="300"></video><br>

The controller was optimized to produce forward propulsion by applying coordinated forces across all four legs. The optimization process tuned the CPG frequency parameters (`f0`, `f1`) and force amplitudes (`Fx`, `Fz`) to achieve maximum forward distance.

### Prerequisites      <a href="videos/Ccw_twist_jump.mkv">📥 Télécharger</a><br>

**Result Video**:

      <em>Twist CCW</em>

[Download Forward Jump Video](videos/Front_jump.mkv)

- Python 3.9 or higher    </td>

**Key Parameters**:

- Impulse frequency (f0): ~1.67 Hz- Conda (recommended) or virtualenv    <td align="center">

- Recovery frequency (f1): ~0.71 Hz

- Forward force amplitude (Fx): ~292 N- Git      <b>🟢 Rotation horaire</b><br>

- Vertical force amplitude (Fz): ~394 N

      <video src="https://github.com/votre-username/Legged_Robots_MR/raw/main/videos/Cw_twist_jump.mkv" width="300"></video><br>

---

### Setting Up the Conda Environment      <a href="videos/Cw_twist_jump.mkv">📥 Télécharger</a><br>

### 3.2 Speed Hopping

      <em>Twist CW</em>

**Objective**: Achieve rapid, continuous jumping with high frequency.

#### Step 1: Clone the Repository    </td>

This behavior focuses on quick successive jumps, useful for fast locomotion over flat terrain. The optimization balanced jump height with frequency to maintain continuous contact-free flight phases.

  </tr>

**Result Video**:

```bash</table>

[Download Speed Hop Video](videos/Speed_hop.mkv)

git clone https://github.com/Emixm4/Legged-Robots-Miniproject.git

---

cd Legged-Robots-Miniproject> **Note** : Les vidéos sont au format MKV. Cliquez sur les liens pour les télécharger et les visualiser localement.

### 3.3 Lateral Jumps

```

**Objective**: Enable side-to-side movements for lateral maneuvering.

---

Lateral jumping is achieved by introducing asymmetric force profiles between left and right legs, creating lateral momentum while maintaining forward stability.

#### Step 2: Create the Conda Environment

#### Left Side Jump

## 🚀 Installation

**Result Video**:

```bash

[Download Left Side Jump Video](videos/Side_jump_L.mkv)

# Create a new conda environment named 'quadruped' with Python 3.9### Prérequis

#### Right Side Jump

conda create -n quadruped python=3.9

**Result Video**:

- **Python 3.9+**

[Download Right Side Jump Video](videos/Side_jump_R.mkv)

# Activate the environment- **Conda** (recommandé) ou **virtualenv**

---

conda activate quadruped- **Git**

### 3.4 Twist Jumps

```

**Objective**: Perform rotational movements while jumping.

### Configuration de l'environnement virtuel Conda

Twist jumps introduce yaw rotation during the flight phase by applying differential forces across diagonal leg pairs. This enables the robot to change orientation while maintaining forward or stationary motion.

**Important**: Always use a dedicated virtual environment to avoid dependency conflicts!

#### Counter-Clockwise Twist

#### 1️⃣ Cloner le dépôt

**Result Video**:

#### Step 3: Install Dependencies

[Download CCW Twist Video](videos/Ccw_twist_jump.mkv)

```bash

#### Clockwise Twist

```bashgit clone https://github.com/votre-username/Legged_Robots_MR.git

**Result Video**:

# Install required Python packagescd Legged_Robots_MR

[Download CW Twist Video](videos/Cw_twist_jump.mkv)

pip install -r requirements.txt```

---

```

### Performance Summary

#### 2️⃣ Créer l'environnement Conda

All jumping behaviors were successfully optimized and demonstrated stable, repeatable performance in simulation. The CPG-based approach proved effective for generating coordinated multi-leg jumping motions, and the Optuna optimization framework efficiently explored the parameter space to find high-performing configurations.

**If PyBullet installation fails** (compilation issues):

**Note**: Videos are in MKV format. Download them to view locally if your browser doesn't support inline playback.

```bash

---

```bash# Créer un nouvel environnement nommé 'quadruped' avec Python 3.9

## 4. Project Structure

# Install PyBullet via conda firstconda create -n quadruped python=3.9

```

Legged-Robots-Miniproject/conda install conda-forge::pybullet

│

├── env/                          # Simulation library (DO NOT MODIFY)# Activer l'environnement

│   ├── __init__.py

│   ├── simulation.py             # QuadSimulator class# Then reinstall other dependenciesconda activate quadruped

│   └── utils.py                  # Utility functions

│pip install -r requirements.txt```

├── a1_description/               # URDF description of Unitree A1 robot

│   ├── config.yaml               # Robot configuration```

│   ├── meshes/                   # 3D mesh files

│   └── urdf/                     # URDF robot model files> 💡 **Conseil** : Utilisez toujours un environnement virtuel dédié pour éviter les conflits de dépendances !

│

├── videos/                       # Result demonstration videos#### Step 4: Verify Installation

│   ├── Front_jump.mkv

│   ├── Speed_hop.mkv#### 3️⃣ Installer les dépendances

│   ├── Side_jump_L.mkv

│   ├── Side_jump_R.mkv```bash

│   ├── Ccw_twist_jump.mkv

│   └── Cw_twist_jump.mkv# Test that everything works```bash

│

├── quadruped_jump.py             # Main jumping controllerpython quadruped_jump.py# Installer les packages Python requis

├── quadruped_jump_opt.py         # Optimization script

├── profiles.py                   # CPG force profile generation```pip install -r requirements.txt

├── requirements.txt              # Python dependencies

└── README.md                     # This file```

```

If successful, a PyBullet simulation window should open showing the quadruped robot!

---

⚠️ **Si PyBullet échoue lors de l'installation** (problèmes de compilation) :

## 5. Usage

### Alternative: Using virtualenv

### Running the Controller

```bash

To run a pre-configured jumping behavior:

```bash# Installer PyBullet via conda

```bash

# Activate the conda environment# Create virtual environmentconda install conda-forge::pybullet

conda activate quadruped

python -m venv venv

# Run the jumping controller

python quadruped_jump.py# Réinstaller les autres dépendances

```

# Activate the environmentpip install -r requirements.txt

The script will launch a PyBullet visualization window showing the robot performing the selected jump type.

# On Windows PowerShell:```

### Running the Optimization

venv\Scripts\Activate.ps1

To optimize controller parameters for a specific objective:

# On Windows CMD:#### 4️⃣ Vérifier l'installation

```bash

# Activate the conda environmentvenv\Scripts\Activate.bat

conda activate quadruped

# On Linux/Mac:```bash

# Run the optimization framework

python quadruped_jump_opt.pysource venv/bin/activate# Tester que tout fonctionne

```

python quadruped_jump.py

The optimization process uses Optuna to search for optimal CPG parameters:

- `f0`: Frequency during impulse phase (Hz)# Install dependencies```

- `f1`: Frequency between impulses (Hz)

- `Fx`, `Fy`, `Fz`: Force amplitudes in X, Y, Z directions (N)pip install -r requirements.txt



### Recording Videos```Si tout est bien installé, une fenêtre de simulation PyBullet devrait s'ouvrir avec le robot quadrupède ! 🎉



To record your own videos, modify the simulation options in `quadruped_jump.py`:



```python---### Alternative : Environnement virtualenv

sim_options = SimulationOptions(

    on_rack=False,           # Robot on ground (not suspended)

    render=True,             # Enable visualization

    record_video=True,       # Enable video recording## Project Structure```bash

    tracking_camera=True,    # Camera follows robot

)# Créer l'environnement

```

```python -m venv venv

Videos will be automatically saved to the `videos/` directory.

Legged-Robots-Miniproject/

---

│# Activer l'environnement

## 6. References

├── env/                          # Simulation library (DO NOT MODIFY)# Windows PowerShell

[1] G. Bellegarda, M. Shafiee, M. E. Özberk and A. Ijspeert, "Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 1443-1450.

│   ├── __init__.pyvenv\Scripts\Activate.ps1

[2] Akiba, T., Sano, S., Yanase, T., Ohta, T., and Koyama, M., "Optuna: A Next-generation Hyperparameter Optimization Framework," in Proceedings of the 25th ACM SIGKDD International Conference on Knowledge Discovery & Data Mining, 2019, pp. 2623–2631.

│   ├── simulation.py             # QuadSimulator class# Windows CMD

---

│   └── utils.py                  # Utility functionsvenv\Scripts\Activate.bat

## 7. License

│# Linux/Mac

This project was developed as part of the Legged Robots course. The code is provided for educational and research purposes.

├── a1_description/               # URDF description of Unitree A1 robotsource venv/bin/activate

---

│   ├── config.yaml

## 8. Contact

│   ├── meshes/                   # 3D mesh files# Installer les dépendances

For questions, issues, or suggestions, please open an issue on the GitHub repository.

│   └── urdf/                     # URDF filespip install -r requirements.txt

---

│```

**Developed for Legged Robotics Research**

├── videos/                       # Result videos

│   ├── Front_jump.mkv---

│   ├── Speed_hop.mkv

│   ├── Side_jump_L.mkv## 📁 Structure du projet

│   ├── Side_jump_R.mkv

│   ├── Ccw_twist_jump.mkv```

│   └── Cw_twist_jump.mkvLegged_Robots_MR/

││

├── quadruped_jump.py             # Main controller script├── 📂 env/                          # Code de la simulation (NE PAS MODIFIER)

├── quadruped_jump_opt.py         # Optimization script│   ├── __init__.py

├── profiles.py                   # Force profile generation│   ├── simulation.py                # Classe QuadSimulator

├── requirements.txt              # Python dependencies│   └── utils.py                     # Utilitaires

└── README.md                     # This file│

```├── 📂 a1_description/               # Description URDF du robot A1

│   ├── config.yaml

### Key Files│   ├── meshes/                      # Modèles 3D

│   └── urdf/                        # Fichiers URDF

| File | Description |│

|------|-------------|├── 📂 videos/                       # Vidéos de résultats

| `quadruped_jump.py` | Main jumping controller implementation |│   ├── Front_jump.mkv

| `profiles.py` | CPG oscillator-based force profile generation |│   ├── Speed_hop.mkv

| `quadruped_jump_opt.py` | Parameter optimization using Optuna |│   ├── Side_jump_L.mkv

| `env/simulation.py` | PyBullet simulation interface |│   ├── Side_jump_R.mkv

│   ├── Ccw_twist_jump.mkv

---│   └── Cw_twist_jump.mkv

│

## Usage├── 📄 quadruped_jump.py            # ⭐ Script principal du contrôleur

├── 📄 quadruped_jump_opt.py        # ⭐ Script d'optimisation

### Running a Simulation├── 📄 profiles.py                  # ⭐ Profils de force

├── 📄 requirements.txt              # Dépendances Python

```bash└── 📄 README.md                     # Ce fichier

# Activate the conda environment```

conda activate quadruped

### Fichiers principaux

# Run the jumping controller

python quadruped_jump.py| Fichier | Description |

```|---------|-------------|

| `quadruped_jump.py` | Implémentation du contrôleur de saut principal |

The simulation will display the robot performing jumps in the PyBullet GUI.| `profiles.py` | Génération des profils de force via oscillateurs CPG |

| `quadruped_jump_opt.py` | Optimisation des paramètres avec Optuna |

### Optimizing Parameters| `env/simulation.py` | Interface de simulation PyBullet |



```bash---

# Run the optimization framework

python quadruped_jump_opt.py## 💻 Utilisation

```

### Lancer une simulation

The optimization process tunes the following parameters:

- `f0`: Frequency during impulse phase (Hz)```bash

- `f1`: Frequency between impulses (Hz)# Activer l'environnement

- `Fx`, `Fy`, `Fz`: Force amplitudes (N)conda activate quadruped



### Recording Videos# Lancer le contrôleur de saut

python quadruped_jump.py

To record a video of the simulation, modify the simulation options in `quadruped_jump.py`:```



```python### Optimiser les paramètres

sim_options = SimulationOptions(

    on_rack=False,```bash

    render=True,# Lancer l'optimisation avec Optuna

    record_video=True,      # Enable video recordingpython quadruped_jump_opt.py

    tracking_camera=True,```

)

```L'optimisation cherche les meilleurs paramètres pour maximiser la distance de saut en ajustant :

- `f0` : Fréquence pendant l'impulsion (Hz)

Videos will be saved to the `videos/` directory.- `f1` : Fréquence entre les impulsions (Hz)

- `Fx`, `Fy`, `Fz` : Amplitudes des forces (N)

---

### Enregistrer une vidéo

## VS Code Extensions

Dans `quadruped_jump.py`, modifiez les options de simulation :

For an optimal development experience, we recommend installing the following VS Code extensions:

```python

### Essential Extensionssim_options = SimulationOptions(

    on_rack=False,

1. **Python** (`ms-python.python`)    render=True,

   - Full Python language support    record_video=True,      # ✅ Activer l'enregistrement

   - Debugging, linting, and formatting    tracking_camera=True,

)

2. **Pylance** (`ms-python.vscode-pylance`)```

   - Fast, feature-rich language support

   - Type checking and IntelliSenseLes vidéos seront sauvegardées dans le dossier `videos/`.



3. **Jupyter** (`ms-toolsai.jupyter`)---

   - Notebook support for data analysis

## 🔧 Extensions VS Code recommandées

### Recommended Extensions

Pour une meilleure expérience de développement, installez ces extensions VS Code :

4. **autoDocstring** (`njpwerner.autodocstring`)

   - Automatic docstring generation### Essentielles



5. **Better Comments** (`aaron-bond.better-comments`)1. **Python** (`ms-python.python`)

   - Enhanced comment highlighting   - Support complet pour Python

   - Débogage, linting, formatage

6. **GitLens** (`eamodio.gitlens`)   

   - Git supercharged features2. **Pylance** (`ms-python.vscode-pylance`)

   - Autocomplétion intelligente

7. **Python Indent** (`KevinRose.vsc-python-indent`)   - Vérification de types

   - Improved auto-indentation

3. **Jupyter** (`ms-toolsai.jupyter`)

8. **Markdown All in One** (`yzhang.markdown-all-in-one`)   - Pour les notebooks si vous faites de l'analyse

   - Enhanced Markdown support

### Recommandées

### Quick Installation

4. **autoDocstring** (`njpwerner.autodocstring`)

Install all recommended extensions at once:   - Génération automatique de docstrings



```bash5. **Better Comments** (`aaron-bond.better-comments`)

code --install-extension ms-python.python   - Coloration des commentaires pour TODO, FIXME, etc.

code --install-extension ms-python.vscode-pylance

code --install-extension ms-toolsai.jupyter6. **GitLens** (`eamodio.gitlens`)

code --install-extension njpwerner.autodocstring   - Historique Git enrichi

code --install-extension aaron-bond.better-comments

code --install-extension eamodio.gitlens7. **Python Indent** (`KevinRose.vsc-python-indent`)

code --install-extension KevinRose.vsc-python-indent   - Indentation automatique améliorée

code --install-extension yzhang.markdown-all-in-one

```8. **Markdown All in One** (`yzhang.markdown-all-in-one`)

   - Support enrichi pour Markdown

### Workspace Configuration

### Installation rapide

The project includes VS Code workspace settings that automatically configure:

- Python interpreter pathVous pouvez installer toutes les extensions recommandées en une seule commande :

- Code formatting rules

- Linting preferences```bash

- File exclusions for cleaner workspace# Dans PowerShell

code --install-extension ms-python.python

---code --install-extension ms-python.vscode-pylance

code --install-extension ms-toolsai.jupyter

## Technical Detailscode --install-extension njpwerner.autodocstring

code --install-extension aaron-bond.better-comments

### Controller Architecturecode --install-extension eamodio.gitlens

code --install-extension KevinRose.vsc-python-indent

The jumping controller uses a CPG (Central Pattern Generator) oscillator to create rhythmic force patterns:code --install-extension yzhang.markdown-all-in-one

```

- **Impulse Phase**: High force application to propel the robot

- **Recovery Phase**: Low force period between jumps### Configuration VS Code

- **Force Profiles**: Customizable X, Y, Z force components for different behaviors

Créez un fichier `.vscode/settings.json` pour configurer votre environnement :

### Optimization Strategy

```json

The Optuna-based optimization framework:{

- Uses Tree-structured Parzen Estimator (TPE) sampling    "python.defaultInterpreterPath": "${workspaceFolder}/venv/Scripts/python.exe",

- Maximizes objective functions (e.g., distance traveled)    "python.linting.enabled": true,

- Automatically explores parameter space    "python.linting.pylintEnabled": true,

- Provides visualization of optimization history    "python.formatting.provider": "black",

    "editor.formatOnSave": true,

### Simulated Robot    "files.autoSave": "afterDelay",

    "autoDocstring.docstringFormat": "numpy"

The **Unitree A1** quadruped robot features:}

- 12 degrees of freedom (3 per leg)```

- Accurate mass and inertia properties

- High-fidelity collision geometry---

- Realistic joint limits and dynamics

## 📦 Dépendances

---

Le projet utilise les packages Python suivants :

## Dependencies

| Package | Version | Description |

The project requires the following Python packages:|---------|---------|-------------|

| **numpy** | 1.26.4 | Calculs numériques et algèbre linéaire |

| Package | Version | Purpose || **pybullet** | 3.2.7 | Moteur de simulation physique 3D |

|---------|---------|---------|| **PyYAML** | 6.0.2 | Parsing de fichiers de configuration YAML |

| **numpy** | 1.26.4 | Numerical computations and linear algebra || **optuna** | 4.5.0 | Framework d'optimisation bayésienne |

| **pybullet** | 3.2.7 | 3D physics simulation engine |

| **PyYAML** | 6.0.2 | YAML configuration file parsing |### Installer les dépendances

| **optuna** | 4.5.0 | Bayesian optimization framework |

Toutes les dépendances sont listées dans `requirements.txt` :

### Installation

```txt

All dependencies are listed in `requirements.txt`:numpy==1.26.4

pybullet==3.2.7

```txtPyYAML==6.0.2

numpy==1.26.4optuna==4.5.0

pybullet==3.2.7```

PyYAML==6.0.2

optuna==4.5.0Installation :

```

```bash

Install with:pip install -r requirements.txt

```

```bash

pip install -r requirements.txt---

```

## 🧪 Détails techniques

---

### Contrôleur de saut

## References

Le contrôleur utilise un oscillateur CPG (Central Pattern Generator) pour générer des profils de force périodiques :

```bibtex

[1] G. Bellegarda, M. Shafiee, M. E. Özberk and A. Ijspeert, - **Phase d'impulsion** : Application d'une force importante pour propulser le robot

    "Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping," - **Phase de repos** : Récupération entre les sauts

    2024 IEEE International Conference on Robotics and Automation (ICRA), - **Profils de force** : Combinaison de forces X, Y, Z pour différentes trajectoires

    Yokohama, Japan, 2024, pp. 1443-1450.

### Paramètres optimisés

[2] Akiba, T., et al, 

    "Optuna: A Next-generation Hyperparameter Optimization Framework," L'optimisation par Optuna permet de trouver les meilleurs paramètres pour :

    in Proceedings of the 25th ACM SIGKDD International Conference on 

    Knowledge Discovery & Data Mining, 2019, pp. 2623–2631.- Maximiser la distance parcourue

```- Réaliser des sauts latéraux précis

- Effectuer des rotations contrôlées

---- Maintenir la stabilité du robot



## License### Robot simulé



This project was developed as part of a Legged Robots course.Le robot utilisé est le **Unitree A1**, un quadrupède avec :

- 12 actionneurs (3 par patte)

---- Modèle URDF complet avec masses et inerties

- Simulation PyBullet à haute fidélité

## Contributing

---

Contributions are welcome! Feel free to:

- Open an issue to report bugs## 📚 Références

- Submit pull requests to improve the code

- Share your optimization results```bibtex

[1] G. Bellegarda, M. Shafiee, M. E. Özberk and A. Ijspeert, 

---    "Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping," 

    2024 IEEE International Conference on Robotics and Automation (ICRA), 

## Contact    Yokohama, Japan, 2024, pp. 1443-1450.



For questions or suggestions, please open an issue on GitHub.[2] Akiba, T., et al, 

    "Optuna: A Next-generation Hyperparameter Optimization Framework," 

---    in Proceedings of the 25th ACM SIGKDD International Conference on 

    Knowledge Discovery & Data Mining, 2019, pp. 2623–2631.

**Developed for Legged Robotics Research**```



If you find this project helpful, please consider giving it a star!---


## 📝 Licence

Ce projet a été développé dans le cadre d'un cours sur les robots à pattes (Legged Robots). 

---

## 🤝 Contribution

Les contributions sont les bienvenues ! N'hésitez pas à :

- Ouvrir une **issue** pour signaler un bug
- Proposer une **pull request** pour améliorer le code
- Partager vos résultats et optimisations

---

## 📞 Contact

Pour toute question ou suggestion, n'hésitez pas à me contacter ou à ouvrir une issue sur GitHub.

---

<div align="center">

**Développé avec ❤️ pour la robotique à pattes**

⭐ Si ce projet vous a plu, n'hésitez pas à lui donner une étoile !

</div>
