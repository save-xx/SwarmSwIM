# SwarmSwIM: an Underwater Swarm Simulator 
This is a Python3-based simulator designed for modeling multi-robot and swarm systems. It implements a simplified motion model, with an assumed level of low level control already acting on the simulated agents rather than calculating the full-body dynamics. This approach allows the simulator to efficiently handle a large number of agents simultaneously.

**Related repositories**
- [swarmswim examples](https://github.com/save-xx/swarmswim_examples)
- [swarmswimros](https://github.com/save-xx/swarmswimros)

## Features

- **Efficient Swarm Simulation**: Simulate numerous agents with simplified motion models.
- **Modular and Customizable**: Easily extendable for different agent types and behaviors.
- **Scalability**: Optimized for handling multiple agents.
- **Basic Visualizations**: Includes simple tools for visualizing agent positions and swarm dynamics.

## Requirements

The following Python packages are required to run the simulator:

- **Core Functionality**: `numpy`
- **Visualization & Animation**: `matplotlib`

Additionally **SimAPI** requires additional packages to interconnect with UE5:  

- **API functionality**: `fastapi`, `uvicorn`
- **Image processing**: `opencv-python`


Note that the installation will handle all the dependacy installations.

## Installation
The package can be installed Globally to be accessible regardless the location.

---
#### Generic installation
For a generic installation, just run: 
```bash
pip install git+https://github.com/save-xx/SwarmSwIM.git
```
Note that you will not be able to alter the code.

---
#### Dev installation
For a developer (editable) installation:

Clone the repository in a Folder of your choice:
```bash
git clone https://github.com/save-xx/SwarmSwIM.git
```

Enter the folder that has been cloned
in the `SwarmSwIM` folder:  
```bash
cd SwarmSwIM/ # Go to download folder
```
  
Install SwarmSwIM as python pakage, `-e` indicates that is editable.
```bash
pip install -e .
```

The installation should conclude with:
```
Successfully built SwarmSwIM
Installing collected packages: SwarmSwIM
Successfully installed SwarmSwIM-x.x.x
```

---
#### Uninstall

If you want to install the dependancies manually:

```bash
pip install numpy matplotlib
pip install fastapi uvicorn opencv-python
```


To Uninstall the package simply:
```bash
pip uninstall SwarmSwIM
```
> Note: Dependace installed with the package will not be automatically unistalled (`numpy`, `matplotlib`, etc..). Due to `opencv-python`, the installation will require `numpy` < 2.0.

<!-- 
LEGACY
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
```  -->

## Basic Usage: Using the Simulator as a Python Library
The simulator can be directlely be used as python library to set up your own simulator.
To Set you own simulation:


### Simulation Example 
```python
from SwarmSwIM import Simulator, CNNDetection
TIME_STEP = 1 / 24  # Simulation time step in seconds

# Initialize the simulator with the chosen time step
S = Simulator(TIME_STEP)

# Initialize a sensor model (e.g., CNN-based detection)
Detection = CNNDetection()

# Run the simulation until a specific condition is met
for i in range(1000):
    S.tick()         # Progress the simulation by one time step
    Detection(S)     # Apply detection on the current state of the simulator, return names of agents activated

```

This code runs the simulator in discrete steps until the specified condition is met. Note that the simulator does not run in real time; it iterates through each time step as quickly as possible. This setup is particularly useful for post-analysis and machine learning applications, where real-time processing is not required, and fast iteration are preferrable.

### Minimal animation
The animator2D provides a simple representation of the agents on the planar position and heading of each agent. Within comutational capabilites of the hosting machine and simulation complexity, the animator will run the simulation in real-time. A code example of the minimal animation can be found in the file `example2Danimation.py`.

With the Package installed, from the SwarmSwIM folder, launch the example:
```bash
python3 -m SwarmSwIM.example2Danimation
```

## More Examples
In the [swarmswim examples](https://github.com/save-xx/swarmswim_examples) are stored more examples, showcasing the different functionalities as well as full case scenarios and examples of uses of SwarmSwIM.

## ROS2 Implementation
This simulator is also avaiable for ROS2 implementation. The ROS2 impementation is designed as a stand-alone module based on this core. It will __not__ require the core installation, since it is already designed with the simulator locally in-build. For information on the installation and use please check-out the ros2 swarmswim repo at:  [swarmswimros](https://github.com/save-xx/swarmswimros)

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
 
[UE5 SwarmSim for Windows](https://drive.google.com/uc?export=download&id=1Jm6av7dfeh0tLGYfWLKlqPPpbLTuIgow)  

[UE5 SwarmSim for Linux (Ubuntu)](https://drive.google.com/uc?export=download&id=1bLFngBHwfXkdiB3LFTvfcEyZxp3wsc5J)

[UE5 SwarmSim for Linux (OLD)](https://drive.google.com/uc?export=download&id=16G_9QRhCBSX7UGSws5W-YAe6_rQ6eY0m) 

## Wiki
For more details, check out the [Wiki](https://github.com/save-xx/SwarmSwIM/wiki).
