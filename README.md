# QuadRobotParallelSim 🤖🐾

**QuadRobotParallelSim** is a high-performance Python-based simulation framework for trajectory planning and optimization of quadruped robots. Leveraging shared memory parallelism, this project accelerates simulation speed and enhances performance, enabling real-time trajectory tracking and visualization using PyBullet and Matplotlib.

![vid1](/img/vid1.gif)

## Table of Contents 📚
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Methodology](#methodology)
- [Performance Evaluation](#performance-evaluation)


## Overview 🌍
Quadruped robots are essential for navigating complex terrains in applications like search-and-rescue and agricultural automation. This project explores the benefits of parallelizing trajectory simulations to optimize performance using Python's multiprocessing capabilities. Key goals include:
- Simulating multiple quadrupeds with independent trajectory planning.
- Introducing realistic movement through oval trajectories with jitter.
- Visualizing real-time leg trajectories in 3D.

The project was developed as part of the research **"How Can Shared Memory Parallelism Improve Simulation Speed and Overall Performance in Quadruped Robot Trajectory Planning?"**


## Features 🚀
- **Parallelized Simulations:** Run multiple PyBullet clients in parallel using Python's `multiprocessing` library.
- **Oval Trajectory Planning:** Generate smooth, oval-shaped trajectories with customizable parameters like radius, height, and jitter for added unpredictability.
- **3D Real-Time Visualization:** Plot leg trajectories in real-time with Matplotlib.
- **Interactive GUI:** Adjust trajectory parameters dynamically during the simulation via PyBullet's GUI sliders.
- **Performance Benchmarking:** Compare execution times between parallel and serial simulations by running `parallel_log.py/serial_log.py`, and plot with `plot_logs.py`.



## Installation 🛠️
### Prerequisites
- [Python 3.8+](https://www.python.org/) 🐍
- [PyBullet](https://pypi.org/project/pybullet/) ⚙️
- [Matplotlib](https://matplotlib.org/) 📊
- [Numpy](https://numpy.org/) 🔢
- [SciPy](https://www.scipy.org/)
- [SymPy](https://www.sympy.org/)

### Steps
1. 🌀 Clone the repository:
   ```bash
   git clone https://github.com/rorosaga/QuadRobotParallelSim
   cd QuadRobotParallelSim
   ```
2. 📦 Install the required packages: 
   ```bash
   pip install -r requirements.txt
   ```

## Usage 🎮️
🏃‍♀️ Run the parallel simulation:
   ```bash
   cd src
   python final_parallel.py
   ```
📊 Plot the performance logs:
   ```bash
   python parallel_log.py
   python serial_log.py
   python plot_logs.py
   ```

## Methodology 🔬

🛠️ Each robot is initialized with independent PyBullet clients.
Trajectories are generated using oval equations with added jitter for realism. 🛠️

⚡Trajectory computations for multiple quadrupeds are parallelized across CPU cores.⚡

👀 Shared queues handle inter-process communication for real-time plotting. 👀

🎥 Leg positions are plotted in 3D using Matplotlib, showing dynamic updates in real-time. 🎥

## Performance Evaluation ⚖️


| No. Simulations | Serial Time (s) | Parallel Time (s) | SpeedUp   |
|-----------------|-----------------|-------------------|-----------|
| 3               | 9.32            | 3.22              | 2.89x     |
| 4               | 13.68           | 5.03              | 2.72x     |
| ...             | ...             | ...               | ...       |

![speedup](/img/execution_times_comparison.png)

## Acknowledgments 

This project is based on [LeggedRobotsForBullet]([https://github.com/original-author/repo-name](https://github.com/haruki1526/LeggedRobotsForBullet)) by Haruki Takamura. The original project is licensed under the MIT License.


**Don't hesitate to reach out if you have any questions or suggestions! 🦾**
