import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import numpy as np
import random

# number of elements preserved in the tail
THRESHOLD = 300

class Plotter:
    def __init__(self,simulator,SIZE=30,artistics=[]):
        # load list of agents
        self.sim = simulator
        # Set plot axis NED coordinate system
        self.fig2, self.ax = plt.subplots()
        self.ax.set_xlim(-SIZE, SIZE)  # Set the x-axis limits
        self.ax.set_ylim(SIZE, -SIZE)  # Set the y-axis limits
        self.ax.axis('equal')
        # Add background features, if any
        for art in artistics:
            self.ax.add_artist(art)
        # initialize memory
        self.animation = {}

        for i,agent in enumerate(self.sim.agents):
            color = plt.get_cmap('tab20')(i % 20)
            self.add(agent, color)

    def add(self,agent,color):
        ''' add animation parameters '''
        # Initialize the data arrays for x and y
        x_data = np.array([agent.pos[0]])
        y_data = np.array([agent.pos[1]])
        line, = self.ax.plot([],[],color=color)
        poly = Polygon(np.array([[0, 0], [0, 0], [0, 0]]), closed=True, color= color, zorder=10)
        self.ax.add_patch(poly)
        self.animation[agent.name] = {'x': x_data, 'y': y_data, 'line': line, 'figure': poly} 

    def remove(self,agent):
        if agent.name in self.animation: del self.animation[agent.name]

    def check_agents(self):
        ''' add or remove additional agents with ongoing simulation'''
        # add new agent
        for agent in self.sim.agents:
            if not agent.name in self.animation:
                self.add(agent,(random.random(),random.random(),random.random(),1.0))
        # remove no longer present agent check fast
        # if len(self.animation)==len(self.sim.agents): return
        # remove if different
        temporary_namelist = [ag.name for ag in self.sim.agents]
        keys_to_remove= []
        for key in self.animation:             
            if not key in temporary_namelist: keys_to_remove.append(key)
        self.detections = {key: self.animation[key] for key in self.animation if key not in keys_to_remove}


    def update_plot(self,callback=None):
        def update(frame):
            self.check_agents()
            artist_list = []
            for agent in self.sim.agents:
                # add position to list
                self.animation[agent.name]['x'] = np.append(self.animation[agent.name]['x'], agent.pos[0])
                self.animation[agent.name]['y'] = np.append(self.animation[agent.name]['y'], agent.pos[1])
                # Pop excess
                if len(self.animation[agent.name]['x'])>THRESHOLD: self.animation[agent.name]['x'] = np.delete(self.animation[agent.name]['x'], 0)
                if len(self.animation[agent.name]['y'])>THRESHOLD: self.animation[agent.name]['y'] = np.delete(self.animation[agent.name]['y'], 0)
                # Update the plot lines paths
                self.animation[agent.name]['line'].set_data(self.animation[agent.name]['x'], self.animation[agent.name]['y'])
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
        sin = np.sin(np.deg2rad(agent.psi))
        cos = np.cos(np.deg2rad(agent.psi))
        xg,yg = agent.pos[0],agent.pos[1]

        x1 = [xg +0.2*cos         , yg +0.2*sin       ]
        x2 = [xg -0.2*cos -0.1*sin, yg+0.1*cos-0.2*sin]
        x3 = [xg -0.2*cos +0.1*sin, yg-0.1*cos-0.2*sin]
        return np.array([x1,x2,x3])
        


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



# ## LEGACY

        # # initalization
        # self.lines_list = []
        # self.tri_list = []
        # self.xdata_list = []
        # self.ydata_list = []

#     def add_agent(self,agent,color):
#         # Initialize the data arrays for x and y
#         x_data = np.array([agent.pos[0]])
#         y_data = np.array([agent.pos[1]])
#         self.xdata_list.append(x_data)
#         self.ydata_list.append(y_data)
#         # Intialize plot
#         line, = self.ax.plot([],[],color=color)
#         self.lines_list.append(line)
#         # Initialize Polygon
#         poly = Polygon(np.array([[0, 0], [0, 0], [0, 0]]), closed=True, color= color, zorder=10)
#         self.ax.add_patch(poly)
#         self.tri_list.append(poly)


    # def update_plot(self,callback=None):
    #     def update(frame):
    #         for i, agent in enumerate(self.Agents):
    #             # add position to list
    #             self.xdata_list[i] = np.append(self.xdata_list[i], agent.pos[0])
    #             self.ydata_list[i] = np.append(self.ydata_list[i], agent.pos[1])
    #             # Pop excess
    #             if len(self.xdata_list[i])>THRESHOLD: self.xdata_list[i] = np.delete(self.xdata_list[i], 0)
    #             if len(self.ydata_list[i])>THRESHOLD: self.ydata_list[i] = np.delete(self.ydata_list[i], 0)
    #             # Update the plot lines paths
    #             self.lines_list[i].set_data(self.xdata_list[i], self.ydata_list[i])
    #             # Update the polygon coordinates
    #             pts = self.calculate_triangle(agent)
    #             self.tri_list[i].set_xy(pts)
    #         self.ax.relim()
    #         #self.ax.autoscale_view()
    #         return  self.tri_list + self.lines_list 