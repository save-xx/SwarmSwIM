# Underwater Swarm Simulator 
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

It is suggested to make sure that the package folder is added to the PYTHONPATH of the system.

For Ubuntu it can be added as:  
Bash:
```bash
echo 'export PYTHONPATH=$PYTHONPATH:/path/to/uw_swarmsim' >> ~/.bashrc
```
Zsh:  
```bash
echo 'export PYTHONPATH=$PYTHONPATH:/path/to/uw_swarmsim' >> ~/.zshrc
```

For Windows:

- Press Win + S and search for "Environment Variables."
- Click "Edit the system environment variables."
- In the System Properties window, click the Environment Variables button.
- In the System Variables section, locate (or create) the PYTHONPATH variable:
    - If PYTHONPATH exists: Select it, click Edit, and add ;C:\path\to\uw_swarmsim to the end.
    - If PYTHONPATH does not exist: Click New, set the name as PYTHONPATH, and value as C:\path\to\uw_swarmsim.
- Click OK on all windows to save the changes.

Or in PowerShell

```bash
[System.Environment]::SetEnvironmentVariable("PYTHONPATH", $env:PYTHONPATH + ";C:\path\to\uw_swarmsim", [System.EnvironmentVariableTarget]::Machine)
``` 

## Basic Usage: Using the Simulator as a Python Library
The simulator can be directlely be used as python library to set up your own simulator.
To Set you own simulation:


### Minimal simulation 
```python
from sim_class import Simulator
time_step = 1 / 24  # Simulation time step in seconds

# Initialize the simulator with the chosen time step
S = Simulator(time_step)

# Initialize a sensor model (e.g., CNN-based detection)
Detection = CNNDetection()

# Run the simulation until a specific condition is met
while condition:
    S.tick()         # Progress the simulation by one time step
    Detection(S)     # Apply detection on the current state of the simulator

```

This code runs the simulator in discrete steps until the specified condition is met. Note that the simulator does not run in real time; it iterates through each time step as quickly as possible. This setup is particularly useful for post-analysis and machine learning applications, where real-time processing is not required, and fast iteration are preferrable.

### Minimal animation
The animator2D provides a simple representation of the agents on the planar position and heading of each agent. Within comutational capabilites of the hosting machine and simulation complexity, the animator will run the simulation in real-time. A code example of the minimal animation can be found in the file `example2Danimation.py`.

## Attributes and Methods

### Simulator
**Parameters**
- **Dt** (float) Simulation interval in seconds (mandatory)
- **sim_xml** (string) Local path to the xml file describing the simulation settings. Default is "simulation.xml"

**Attributes**
- **time**: (float) current enlapsed time in seconds
- **agents**: (list of Agents) list of the actively simulated agents
- **environment**: (dictionary) Collection of all current parameters
- **states**: (dictionary name: np.array[x,y,z]) position of each agent in the simulation, collected by agent name as key 

**Methods**
- **add (*args)**-> None: add one or more agents to the simulation. 
    - args: one or more Agent object
- **remove (*args)**-> None: remove, if present, one or more agents to the simulation. 
    - args: one or more Agent object in agents.
- **tick()**-> None: move the simulation foward of the interval indicated in Dt
- **rel_pos(Agent1,Agent2)**-> numpy.array: return the istantaneous position vector of Agent2 respect to Agent1.
    - Agent1, Agent2: Agent objects
- **acoustic_range(Agent1,Agent2)**-> float: return the distance vector of Agent2 respect to Agent1, considering acoustic delay, including measurament errors.
    - Agent1, Agent object (reciver)
    - Agent2: Agent object (sender)
- **OWTT_acoustic_range(Agent1,Agent2)**-> float:return the distance vector of Agent2 respect to Agent1, considering acoustic delay, , including measurament errors and One way clock drift error.
    - Agent1, Agent object (reciver)
    - Agent2: Agent object (sender)
- **doppler(Agent1,Agent2,msg_dt=1.0)**-> float: return the value of doppler velocity of Agent2 measured by agent1 on an incoming communication, including measurament errors.
    - Agent1, Agent object (reciver)
    - Agent2: Agent object (sender)



### Agent
**Parameters**
- **name**: (string) Unique name of the agent added. Mandatory field
- **Dt** (float) Simulation interval in seconds. Overwritten by simulator. Default 0.0.
- **pos** (array-like[x,y,z]) initial position of the agent in the world, NED coordinated. Default [0,0,0].
- **psi0** (float) initial heading of the agent. NED coordinates. Default 0.0 (North).
- **agent_xml** (string) Local path to the xml file describing the agent settings. Default is "default.xml"

**Attributes**

- **measured_depth**: (float) Measured estimated depth used in depth control
- **measured_heading**: (float) Measured estimated heading used in heading control
- **measured_pos**: (list[x,y]) Measured estimated position used in position control 

- **cmd_depth**: (float) get/set desired depth in meters. Only when depth_control is ideal, step, proportional.
- **cmd_heave**: (float) get/set desired heave velocity. Only when depth_control is heave.
- **cmd_heading**: (float) (float) get/set desired heading in degrees. Only when heading_control is ideal, step, proportional.
- **cmd_yawrate**: (float) get/set desired rotational yaw rate velocity. Only when depth_control is yawrate.
- **cmd_planar**: (numpy.array[x,y]) get/set planar desired coordinates. Only when planar_control is ideal or step.
- **cmd_local_vel**: (array like or float) get/set desired surge and sway velocity in body frame. Only when planar_control is local_velocity. If set float, assumes sway=0.
- **cmd_forces**: (float) (array like or float) get/set desired surge and sway forces in body frame. Only when planar_control is local_forces. If set float, assumes sway=0.

- **internal_clock**: (float) internally measured time, affected by clock drift.

- **NNDetector**: (dictionary) results obtained since last detection event. Each key is the name of a detected agent, and the item associated is a list of [distance, relative horizontal angle, relative vertical angle]. A special key named time_lapsed contains the value of the time interval since last detection.

**Methods**
- **cmd_fhd(force,heading,depth)**->None: Set desired command as force (float or array-like[x,y]), heading (float), depth (float). 

- **cmd_xyz_phi(position,heading)**->None: Set desired waypoint (np.array[3]) and heading (float)

## ROS2 Implementation
Install the core as above and then refer to [swarmsim](https://github.com/save-xx/swarmsim)

## Unreal Engine: Simulation [EXPERIMENTAL - STILL UNDER DEV]
Two files are added to give you the possibility to simulate a 3D marine environment. Use the sample project that is shared in the 'UE5_sim' folder as a sample. 

When creating your own project, keep it simple:
- On the left sidebar, select `Games`, then `Blank`. Keep Blueprint mode!
- After you reach the viewport, with `File` --> `New Level` --> `Empty Level` you will create a new environment
- Switch to `Landscape mode` to create your own landscape. In the `Sculpt` section you can experiment different landscape setups.

`SimAPI.py` is providing an ongoing simulation mode through HTTP Request. On the loopback address, when you request `/init_status` the environment is set up as in the `init_unreal.py` file. During simulaiton, every tick will perform a `/tick_exec` request to update the visual simulation. As a result of this, for each agent simulated, a visual feedback is returned and shown through a OpenCV (python) operation.

Launch as requested here in order to avoid crashing of UE5 environment!
- Launch the `SimAPI.py` file, that provides the link between simulation and visual representation
- Open the UnrealEngine project. In `Plugin`-->`Python`, add the path of the startup script int `Additional Paths`and the name of the script in `Startup Scripts`. In this way, UE5 will automatically launch the `init_unreal.py` script and set up the environment on opening the project.
- Play the simulation in Unreal Engine

### Downloads 
[UE5 SwarmSim for Linux (Ubuntu)](https://drive.google.com/uc?export=download&id=16G_9QRhCBSX7UGSws5W-YAe6_rQ6eY0m)  
[UE5 SwarmSim for Windows](https://drive.google.com/uc?export=download&id=1Jm6av7dfeh0tLGYfWLKlqPPpbLTuIgow)  
