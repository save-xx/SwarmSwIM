from agent_class import Agent
from sim_class import Simulator
from sensors.visual_detection import CNNDetection
from animator2D import Plotter
import matplotlib.pyplot as plt

# adding optinal graphics
circle = plt.Circle((0, 0), 20, color='g', fill=False, alpha=0.5)

# initiate simulator at 24 fps
S = Simulator(1/24) 

# set inital agents command
S.agents[0].cmd_fhd(0.0,0,0)
S.agents[2].cmd_fhd(-0.0,180,0.1)

# initate detection and animator classes
Detection = CNNDetection()
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
    # Add a new agent
    if counter==100: S.add(Agent('B1'))
    # Remove an existing agent
    if counter==200:S.remove(S.agents[1])


    # Print data
    if not counter%24==0: return
    print('Positions:')
    for agent in S.agents: print(f'{agent.name}: '+ 
                                 f'x: {agent.pos[0]:.3f}, '+
                                 f'y: {agent.pos[1]:.3f}, '+
                                 f'z: {agent.pos[2]:.3f}, '+
                                 f'psi: {agent.psi:.1f}')

    print('Detections:')
    # print (Detection.detections)
    print()
    for agent in Detection.detections:  
        print(f'{agent}: < ',end="")
        for key, item in Detection.detections[agent][0].items():
            print(f'| {key}: '+
                  f'dist: {item[0]:.2f}, '+
                  f'alpha: {item[1]:.1f},'+
                  f'beta: {item[2]:.1f}',end="")
        print(" >")
    print("...")

# MAIN
Animation.update_plot(callback=animation_callback)
    