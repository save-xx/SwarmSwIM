# Example file of a SwarmSwIM simulator

from SwarmSwIM import Simulator
from SwarmSwIM import activate_Detector, activate_Acoustic
from SwarmSwIM import activate_Currents, activate_MapSensor
from SwarmSwIM import save_bag
from SwarmSwIM import Visualizer2D

PERIOD = 0.05 

# Start a Simulator instance with a 0.05 s period
S = Simulator(PERIOD)
# save data in a bag
# save_bag(S)

# Activate current effects
activate_Currents(S)

# Activate visual detectoer
activate_Detector(S)

# Activate acoustic channel (return handle to send messagse)
ac_handle = activate_Acoustic(S)

# Activate Map Sensor
# NOTE: the map_filename (path to image) is required.
# activate_MapSensor(S, "bathimetry", "sample.png", scale=0.1)



# define cycle function
def cycle():
    # spin the simulation
    events = S.tick()
    # print all events of the step
    print(events)

    # ========================
    # some user defined events
    # ========================   
    # do every 60 steps
    if S.step_count % 60 == 0:
        # access agent directly by using the agent name
        agent = S['A01'] 

        # sent a message from A01
        ac_handle.send(agent, f"HI from {agent.name}", msg_duration= 0.8)
        
        # change to velocity inertial and set a velocity
        agent.set_VelocityCmd(0.5, mode="inertial_velocity")



        # Command A01 to rotate 45 degrees clockwise
        agent.set_Heading(agent.cmd_heading + 45. % 360, mode="step")

        # alternatively, command the Yawrate directely
        # agent.set_Yawrate(15.0, enforce=True)

# ==============================
# set 2D real time visualization
# ==============================

# define proprieties for visualization
properties = {
    "bg_color": 'w', # background color (white)
    "grid": True, # add grid visualization
    "color_by_type": False, # set legend - False for individual names True for naming by type
    "record": False, # creade video recording
    }

# create visualizer
visualizer = Visualizer2D(S, cycle, properties=properties)
# runs simulation inside the visualizer
visualizer.run()

