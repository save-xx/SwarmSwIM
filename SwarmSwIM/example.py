from SwarmSwIM import Simulator
from SwarmSwIM import activate_Detector, activate_Acoustic
from SwarmSwIM import activate_Currents
from SwarmSwIM import save_bag
from SwarmSwIM import Visualizer2D

# Start a Simulator instance with a 0.05 s period
S = Simulator(0.05)
# enable short term memory (for visualizing path)
S.enable_memory()
# save data in a bag
save_bag(S)

# Activate current effects
activate_Currents(S)
# Activate visual detectoer
activate_Detector(S)
# Activate acoustic channel (return handle to send messagse)
ac_handle = activate_Acoustic(S)

# give one inital command
S.agents['A01'].cmd_forces = 1.0 # newton, command surge force

# define cycle function
def cycle():
    events = S.tick()
    print(events)
    # some user devined events
    # do every 60 steps
    if S.step_count % 60 == 0:
        # sent a message from A01
        agent = S.agents['A01']
        ac_handle.send(agent, f"HI from {agent.name}", msg_duration= 0.8)
        # Command A01 to rotate 30 clockwise
        agent.cmd_heading = (agent.cmd_heading +45.) % 360

# define proprieties for visualization
properties = {
    "bg_color": 'w', # background color (white)
    "grid": True, # add grid visualization
    "color_by_type": False, # set legend - individual names 
    "record": True, # creade video recording
    }

# run combined simulation and visualization
visualizer = Visualizer2D(S, cycle, properties=properties)
visualizer.run()

