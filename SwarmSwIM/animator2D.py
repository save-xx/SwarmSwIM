import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Patch
import matplotlib.cm as cm
import numpy as np
import random
import os


# number of elements preserved in the tail
THRESHOLD = 300

class Plotter:
    def __init__(self,simulator,SIZE=30,artistics=[],color_by_type=False):
        # load list of agents
        self.sim = simulator
        # Set plot axis NED coordinate system
        self.fig2, self.ax = plt.subplots()
        self.ax.set_xlim(-SIZE, SIZE)  # Set the x-axis limits
        self.ax.set_ylim(-SIZE, SIZE)  # Set the y-axis limits
        self.ax.grid(True) # add grid
        self.ax.axis('equal') # fix scale axis as equal
        self.handles = [] # preparel legend
        self.ax.set_xlabel('W - E [m]')
        self.ax.set_ylabel('S - N [m]')
        
        # Add background features, if any
        for art in artistics:
            self.ax.add_artist(art)
        # initialize memory
        self.animation = {}

        # initialize each agent, assicate colors and legend based on desired scheme
        for i, agent in enumerate(self.sim.agents.values()):
            if color_by_type: # color by type (group)
                color = self.type_color(agent)
            else: # each agent its color and legend entry
                color = self.sequential_color(i)
                self.handles.append(Patch(facecolor=color, label=agent.name))
            self.add(agent, color)

        plt.legend(handles=self.handles)
        # NOTE: the legend is curretly static after the creation

    def type_color(self, agent):
        """Return color based on agent type"""
        # initialize dict if does not exist
        if not hasattr(self, "color_map"):
            self.color_map = {}
        agent_type = os.path.splitext(os.path.basename(agent.agent_type))[0]
        # add new color
        if not agent_type in self.color_map:
            new_color = self.sequential_color(len(self.color_map))
            self.color_map[agent_type] = new_color
            self.handles.append(Patch(facecolor=new_color, label=agent_type))
        #return color based on associated agent_type
        return self.color_map[agent_type]

    @staticmethod
    def sequential_color(i: int, total=10, cmap_name="tab20"):
        cmap = cm.get_cmap(cmap_name, total)
        return cmap(i)

    def add(self,agent,color):
        ''' add animation parameters '''
        # Initialize the data arrays for x and y
        x_data = np.array([agent.pos[0]])
        y_data = np.array([agent.pos[1]])
        line, = self.ax.plot([],[],color=color)
        poly = Polygon(
                        np.array([[0, 0], [0, 0], [0, 0]]), 
                        closed=True, 
                        color= color, 
                        zorder=10,
                        label=agent.name
                        )
        self.ax.add_patch(poly)
        self.animation[agent.name] = {'x': x_data, 'y': y_data, 'line': line, 'figure': poly} 

    def remove(self,agent):
        if agent.name in self.animation: del self.animation[agent.name]

    def check_agents(self):
        ''' add or remove additional agents with ongoing simulation'''
        # add new agent
        for agent in self.sim.agents.values():
            if not agent.name in self.animation:
                self.add(agent, self.sequential_color(len(self.sim.agents)+1))
                # TODO: new added element legend update? allow later modifications?
        # remove if different
        temporary_namelist = list(self.sim.agents.keys())
        keys_to_remove= []
        for key in self.animation:             
            if not key in temporary_namelist: keys_to_remove.append(key)
        self.detections = {key: self.animation[key] for key in self.animation if key not in keys_to_remove}


    def update_plot(self,callback=None):
        def update(frame):
            self.check_agents()
            artist_list = []
            for agent in self.sim.agents.values():
                # add position to list
                self.animation[agent.name]['x'] = np.append(self.animation[agent.name]['x'], agent.pos[0])
                self.animation[agent.name]['y'] = np.append(self.animation[agent.name]['y'], agent.pos[1])
                # Pop excess
                if len(self.animation[agent.name]['x'])>THRESHOLD: self.animation[agent.name]['x'] = np.delete(self.animation[agent.name]['x'], 0)
                if len(self.animation[agent.name]['y'])>THRESHOLD: self.animation[agent.name]['y'] = np.delete(self.animation[agent.name]['y'], 0)
                # Update the plot lines paths
                self.animation[agent.name]['line'].set_data(self.animation[agent.name]['y'], self.animation[agent.name]['x'])
                # Update the polygon coordinates
                pts = self.calculate_triangle(agent)
                self.animation[agent.name]['figure'].set_xy(pts)
                # add to artists list
                artist_list.extend([self.animation[agent.name]['line'],self.animation[agent.name]['figure']])
            self.ax.relim()
            #self.ax.autoscale_view()
            return  artist_list # self.tri_list + self.lines_list 

        # get interval for real-time
        interval = max(1,int(self.sim.Dt*1000))
        #return self.lines_list #, p
        ani = FuncAnimation(self.fig2, update, frames=range(10000), interval=interval, blit=True) 
        
        if callback:
            ani.event_source.add_callback(callback)
        plt.show()

    def calculate_triangle(self, agent):
        """Calculate polygon representing an agent on the 2D plane."""
        sin_psi = np.sin(np.deg2rad(agent.psi))
        cos_psi = np.cos(np.deg2rad(agent.psi))
        
        def rotate(x, y):
            """rotate a 2D point of psi"""
            dx, dy = x - cx, y - cy
            xr = cx + cos_psi * dx - sin_psi * dy
            yr = cy + sin_psi * dx + cos_psi * dy
            return (xr, yr)
        
        cx,cy = agent.pos[0],agent.pos[1]
        vertices = [
            (cx + agent.dimentions[0] / 2, cy),            
            (cx - agent.dimentions[0] / 2, cy - agent.dimentions[1] / 2),
            (cx - agent.dimentions[0] / 2, cy + agent.dimentions[1] / 2) 
        ]
        coordinates = [rotate(x, y) for x, y in vertices]
        return np.array([(y, x) for (x, y) in coordinates])
        


if __name__ == "__main__":

    circle = plt.Circle((0, 0), 20, color='g', fill=False, alpha=0.5)

    class Sim:
        def __init__(self,agents):
            self.agents = agents

    class Agent:
        def __init__(self, name, x, y, psi):
            self.name=name
            self.pos = np.array([x,y,0])
            self.psi = psi

    # Create a list of agents
    agents = [Agent('a',0, 0, 0), Agent('b',5, 5 , 90), Agent('c',-5, -5, 180)]
    # Create Simulation
    sim = Sim(agents)
    # Create a Plotter instance
    plotter = Plotter(sim)
