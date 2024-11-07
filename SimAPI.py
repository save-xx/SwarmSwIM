from fastapi import FastAPI
from pydantic import BaseModel
from sim_class import Simulator
import uvicorn, json
import os

DIR_FILE = os.path.dirname(__file__)

# unreal to sim 
app = FastAPI()

#   -> init_unreal
@app.get("/init_status")
async def init_env():
    fish_states = sim.states 
    data = json.dumps(fish_states).encode('utf-8')
    return {'message': 'Simulator INIT: OK', 'data': data}

#   -> tick of simulator, return simulator.states
@app.get("/tick_exec")
async def tick_exec():
    sim.tick()
    fish_states = sim.states 
    return {'message': 'Actor position UPD: OK', 'data': f'{fish_states}'}

if __name__=="__main__":
    sim = Simulator(1/60, sim_xml=os.path.join(DIR_FILE,'simulation.xml'))
    uvicorn.run(app, host='127.0.0.1', port=5555)