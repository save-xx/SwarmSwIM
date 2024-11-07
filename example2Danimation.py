from agent_class import Agent
from sim_class import Simulator
from sensors.visual_detection import CNNDetection
from animator2D import Plotter
import matplotlib.pyplot as plt

circle = plt.Circle((0, 0), 20, color='g', fill=False, alpha=0.5)

S = Simulator(1/24) 
S.agents[0].cmd_fhd(0.0,0,1)
S.agents[2].cmd_fhd(-0.2,180,0.1)

Detection = CNNDetection()
Animation = Plotter(S,artistics=[circle])

counter = 0

def animation_callback():
    global counter
    S.tick()
    Detection(S)
    counter +=1
    # if counter==50: S.add(Agent('B1'))
    # if counter==100:S.remove(S.agents[0])


# for i in range(1):
Animation.update_plot(callback=animation_callback)
    