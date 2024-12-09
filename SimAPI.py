from fastapi import FastAPI, Body, HTTPException, Request
import uvicorn.config
import uvicorn.server
from sim_class import Simulator
import uvicorn, json
import os
import time
import numpy as np
import base64
import cv2

SCALE = 100

def scale_to_UE5(data_in: dict):
    # works to scale up positions of actors
    state = {}
    for key in data_in.keys():
        # scale all values
        state[key] = [elem*SCALE for elem in data_in[key]]
        # scaling back yaw angle 
        state[key][3] /= SCALE
        state[key][3] += 90
    return state

# ------------------------------------------------------------------------------------------

PHOTO_ID = {}
DEBUG = False

# ------------------- STANDARD FUNCTIONS FOR API - UE5 --------------------------------------

def ue5_method_view(**kwargs):
    name = kwargs['name'] 
    img  = kwargs['data']
    if DEBUG:
        cv2.putText(img, f"ID: {PHOTO_ID[name]}", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
        cv2.imshow(f'{name}_FOV', img)
        cv2.waitKey(1)

def ue5_method_echo(**kwargs):
    if DEBUG: print(kwargs['data']/SCALE)

# ---------------------------------- API CLASS ----------------------------------------------

class UE5_API:
    def __init__(self, sim, sim_loopFunc, mode = 'ue5'):

        # sensor -> callable!
        self.mode = mode
        self.sim = sim
        self.sim_loop = sim_loopFunc
        self.app = FastAPI()

    # -------------------------------- API FUNCS -----------------------------------------

        #   -> ping from unreal
        @self.app.get("/ping")
        async def ping():
            return {'message':'OK'}
        
        #   -> init_unreal
        @self.app.get("/init_status")
        async def init_env():
            fish_states = scale_to_UE5(self.sim.states) 
            return {'message': 'Simulator INIT: OK', 'data': f'{fish_states}'}

        #   -> tick of simulator, return simulator.states
        @self.app.get("/tick_exec")
        async def tick_exec():
            # update simulator
            self.sim_loop(self.sim)
            fish_states = scale_to_UE5(self.sim.states) 
            return {'message': 'Actor Position UPD: OK', 'data': f'{fish_states}'}

        #   -> reception of image
        @self.app.put("/{agent_name}/view")
        async def img_view(agent_name: str, data: str = Body(...)):
            try:
                if agent_name not in self.sim.states.keys():
                    raise HTTPException(status_code=404, detail="Agent not found")
                else:
                    if agent_name in PHOTO_ID: PHOTO_ID[agent_name]+=1 
                    else: PHOTO_ID[agent_name] = 0
                nparr = np.frombuffer(base64.b64decode(data), np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                # show all agents' front camera 
                img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
                self.callbacks['view']({'name': agent_name, 'data': img})

            except Exception as ex:
                return {'message': f'Exception: {ex}'}
            
            return {'message': f'{agent_name}/img: delivered'}

        #   -> reception of echosounder
        @self.app.put("/{agent_name}/echo")
        async def echosounder(agent_name: str, data: str = Body(...)):

            value = np.frombuffer(base64.b64decode(data), np.float32)
            self.callbacks['echo']({'name': agent_name, 'data': value})
            return {'message': f'{agent_name}/echo: delivered'}
        
    # ------------------------------------ FUNCS END -----------------------------------------

    async def call(self):
        '''Server managed asynchronously. '''
        config = uvicorn.Config(self.app, host='11.1.0.116', port=5555)
        server = uvicorn.Server(config)
        await server.serve()

    def __call__(self):
        uvicorn.run(self.app, host='127.0.0.1', port=5555)

if __name__=="__main__":

    ## Standalone Imports
    from sim_class import Simulator
    import argparse

    # If exist get the --xml argument to specify the simulation file to use
    parser = argparse.ArgumentParser(description="Simulation xml filename")
    # Add an optional argument
    parser.add_argument('--xml', type=str, default="simulation", 
                        help='Name of the xml file used to set the simulation. Default is "simulation"')
    filename = parser.parse_args().xml+".xml"
    DIR_FILE = os.path.dirname(__file__)
    
    ## -------------------------------------------
    # Initialize simulation, defalut 60 fps
    sim = Simulator(1/60, sim_xml=os.path.join(DIR_FILE,filename)) 

    # create cycle function
    def sim_loop(sim):
        ''' barebone simulation'''
        sim.tick()

    ## ----------------------------------------------
    
    # Initiate and run the API 
    api = UE5_API(sim, sim_loop)
    api()