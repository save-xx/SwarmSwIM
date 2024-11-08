### Underwater Swarm Simulator 
This is a Python3-based simulator designed for modeling multi-robot and swarm systems. It implements a simplified motion model, with an assumed level of low level control already acting on the simulated agents rather than calculating the full-body dynamics. This approach allows the simulator to efficiently handle a large number of agents simultaneously.

## Features

- **Efficient Swarm Simulation**: Simulate numerous agents with simplified motion models.
- **Modular and Customizable**: Easily extendable for different agent types and behaviors.
- **Scalability**: Optimized for handling multiple agents.
- **Basic Visualizations**: Includes simple tools for visualizing agent positions and swarm dynamics.

## Requirements

The following Python packages are required to run the simulator:

- **Core Functionality**: `numpy`
- **Visualization & Animation**: `matplotlib`

Install dependencies with:
```bash
pip install numpy matplotlib
```

## Basic Usage: Using the Simulator as a Python Library
The simulator can be directlely be used as python library to set up your own simulator.
To Set you own simulation:


# minimal simulation 
from sim_class import Simulator
time_step = 1/24 #simulation time step in seconds
S = Simulator(time_step)
Detection = CNNDetection()
while(condition):
    S.tick()
    Detection(S)

This will run the sumulator until the condition is met. Note that the simulator will not run in real time, but will iterate each step within minimum time. This approach is of interest for post-analysis and ML. 

