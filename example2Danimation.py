from agent_class import Agent
from sim_class import Simulator
from sensors.visual_detection import CNNDetection
from sensors.acoustic_comm import AcousticChannel
from animator2D import Plotter
import matplotlib.pyplot as plt

# adding optinal graphics
circle = plt.Circle((0, 0), 20, color='g', fill=False, alpha=0.5)

# initiate simulator at 24 fps
S = Simulator(1/24) 

# set inital agents command
for agent in S.agents:
    agent.cmd_forces= 0.3

# initate detection and animator classes
Detection = CNNDetection()
Acoustic = AcousticChannel()
Animation = Plotter(S,artistics=[circle])

counter = 0

def animation_callback():
    ''' callback function, move the simulation foward 1 step'''
    global counter
    counter +=1
    # advance 1 time step
    S.tick()
    # compute relative detection
    Detection(S)
    Acoustic(S)
    # Add a new agent
    if counter==100: Acoustic.send(S.agents[0],S, duration=1, payload="Hello from A1")

    # Print data
    if not counter%24==0: return
    print('--- Positions: ---')
    for agent in S.agents: print(f'{agent.name}: '+ 
                                 f'x: {agent.pos[0]:.3f}, '+
                                 f'y: {agent.pos[1]:.3f}, '+
                                 f'z: {agent.pos[2]:.3f}, '+
                                 f'psi: {agent.psi:.1f}')

    print('--- Detections: ---')
    for agent in S.agents: 
        if (len(agent.NNDetector)<=1):continue
        print(f'{agent.name} >>')
        for key in agent.NNDetector:
            if key=='time_lapsed': continue
            print(f'    {key} : ',end="") 
            print(f'[ dist: {agent.NNDetector[key][0]:.2f} ',end="") 
            print(f'alpha: {agent.NNDetector[key][1]:.2f} ',end="") 
            print(f'beta: {agent.NNDetector[key][2]:.2f} ]') 
    print()

# MAIN
Animation.update_plot(callback=animation_callback)
    