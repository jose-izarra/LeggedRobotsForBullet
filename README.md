# QuadRobotParallelSim

**QuadRobotParallelSim** is a high-performance Python-based simulation framework for trajectory planning and optimization of quadruped robots. Leveraging shared memory parallelism, this project accelerates simulation speed and enhances performance, enabling real-time trajectory tracking and visualization using PyBullet and Matplotlib.

![vid1](/img/vid1.gif)

## Table of Contents
1. [Overview](#overview)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Methodology](#methodology)
6. [Performance Evaluation](#performance-evaluation)
7. [Acknowledgements](#acknowledgements)
8. [License](#license)

---

## Overview
Quadruped robots are essential for navigating complex terrains in applications like search-and-rescue and agricultural automation. This project explores the benefits of parallelizing trajectory simulations to optimize performance using Python's multiprocessing capabilities. Key goals include:
- Simulating multiple quadrupeds with independent trajectory planning.
- Introducing realistic movement through oval trajectories with jitter.
- Visualizing real-time leg trajectories in 3D.

The project was developed as part of the research **"How Can Shared Memory Parallelism Improve Simulation Speed and Overall Performance in Quadruped Robot Trajectory Planning?"**

---

## Features
- **Parallelized Simulations:** Run multiple PyBullet clients in parallel using Python's `multiprocessing` library.
- **Oval Trajectory Planning:** Generate smooth, oval-shaped trajectories with customizable parameters like radius, height, and jitter.
- **3D Real-Time Visualization:** Plot leg trajectories in real-time with Matplotlib.
- **Interactive GUI:** Adjust trajectory parameters dynamically during the simulation via PyBullet's GUI sliders.
- **Performance Benchmarking:** Compare execution times between parallel and serial simulations.

---

## Installation
### Prerequisites
- Python 3.8+
- [PyBullet](https://pypi.org/project/pybullet/)
- [Matplotlib](https://matplotlib.org/)
- [Numpy](https://numpy.org/)

### Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/rorosaga/QuadRobotParallelSim
   cd QuadRobotParallelSim
