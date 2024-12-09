# from sim_class import Simulator
# from agent_class import Age
import numpy as np

class AcousticChannel:
    def __init__(self):
        # Define a computational delay from the channel check to the wave front departure
        self.COMPUTATIONAL_DELAY = 0.05 # seconds, delay between CA and sending
        self.C = 1500 # speed of sound m/s
        self.MAX_RANGE = 2000  # 2km max range considered
        # distance where ideally collision avoidance (CA) should happen but ...
        #  computation time already started the sending
        self.BUFF_RADIUS = self.C*self.COMPUTATIONAL_DELAY 
        
        # Status of the channel, None if free
        self.channel_status = []

    def send(self, Agent, Sim , duration, payload=""):
        ''' verify for collisions '''
        status = "active"
        idx_collision = None
        time = Sim.time
        for i, event in enumerate(self.channel_status):
            if status=="denied":break # interrupt if message has been denied
            for circle in event['circles']:
                # get distances from center of wave and agent sending
                distance = np.linalg.norm(circle['center']-Agent.pos)
                # check if the back end of wave is passed, no effect.
                if distance < circle['radii'][0]: continue
                # check if collision avoidance stop transmission.
                if distance < circle['radii'][1]-self.BUFF_RADIUS:
                    status = "denied"
                    break
                # else collision may happen
                status = "collision"
                idx_collision = i # store index of event

        # If the communication has been denied return
        if status=="denied": return "denied"

        # If resulting case is collision, update the event associated
        if status =="collision":
            new_circle = {'radii': [0.0,0.0], 'times': [time, time+duration], 'center': Agent.pos}
            self.channel_status[idx_collision]['circles'].append(new_circle)
            self.channel_status[idx_collision]['status'] = "collision"
            self.channel_status[idx_collision]['payload'] = None
            return "sent - collision"

        # If suceesfull the add new event
        if status=="active":
            new_circle = {'radii': [0.0,0.0], 'times': [time, time+duration], 'center': Agent.pos}
            event = {'circles': [new_circle], 'status': "active", 'payload': payload , 'delivered': [Agent]}
            self.channel_status.append(event)
            return "sent"

    def __call__(self, Sim):
        ''' Tick based call to update message '''
        delivered = {}
        # update all wave fronts
        for event in self.channel_status:
            for circle in event['circles']:
                # Front of the comm wave radius
                circle['radii'][1] =          (Sim.time - circle['times'][0]) * self.C  
                # End of the comm wave radius
                circle['radii'][0] = max(0.0, (Sim.time - circle['times'][1]) * self.C) 
        
            # check and return all recived messages, only test yet to deliver
            agents2check = [agent for agent in Sim.agents if agent not in event['delivered']]
            for agent in agents2check:
                # verify if message is recived
                recived=True
                for circle in event['circles']:
                    distance = np.linalg.norm(agent.pos-circle['center'])
                    if distance > circle['radii'][0]: 
                        recived=False
                        break
                if recived: 
                    # record as delivered
                    event['delivered'].append(agent)
                    delivered[agent.name] = event['payload']
        
        # Clean up expired events from the channel
        for event in self.channel_status:
            # Remove expired circles
            event['circles'] = [c for c in event['circles'] if c['radii'][0] <=self.MAX_RANGE]
        # Remove expired events
        self.channel_status = [e for e in self.channel_status if e['circles']]
        return delivered

    def calculate_toa(self,event,distance):
        '''On Succesfull message calcuate the ToA'''
        # handle failed message with empty return
        if not len(event['circles'])==1: return None
        exact_ToF = distance*self.C
        


if __name__ == "__main__":

    # Define a simple Agent class
    class Agent:
        def __init__(self, name, pos):
            self.name = name
            self.pos = np.array(pos)

    # Define a simple Simulation class
    class Sim:
        def __init__(self, time, agents):
            self.time = time
            self.agents = agents

    # Initialize agents
    agent1 = Agent(name="A1", pos=[0, 0, 0])
    agent2 = Agent(name="A2", pos=[500, 0, 0])
    agent3 = Agent(name="A3", pos=[1000, 0, 0])
    
    # Initialize the simulation
    agents = [agent1, agent2, agent3]
    sim = Sim(time=0, agents=agents)
    
    # Initialize the AcousticChannel
    channel = AcousticChannel()

    # Agent1 sends a message
    print (channel.send(agent1, sim, duration=1, payload="Hello from A1"))

    # Update simulation time and check channel updates
    # print (channel(sim))
    for i in range (30):
        sim.time +=0.1
        if i==9: print (channel.send(agent3, sim, duration=1, payload="Hello from A3 - 2"))
        print (channel(sim))
