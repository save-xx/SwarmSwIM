import requests
import unreal
import json

# This code is used to startup the UnrealEngine environment only

SCALE=10
# assets = unreal.EditorAssetLibrary.list_assets("/Game",recursive=True)
# for asset in assets: unreal.log(asset)

ACTOR = unreal.EditorAssetLibrary.load_blueprint_class("/Game/Blueprints/FishActor")
if not ACTOR: raise Exception("ACTOR ERROR")

def open_level():
    level_name = "/Game/Levels/SimulationLevel"
    unreal.EditorLevelLibrary.load_level(level_name)
    all_actors = unreal.EditorLevelLibrary.get_all_level_actors()
    for actor in all_actors:
        if not isinstance(actor, unreal.Landscape): unreal.EditorLevelLibrary.destroy_actor(actor)
    print(f"Level '{level_name}' opened.")

def load_ocean():
    # water body lake params
    water_location = unreal.Vector(0, 0, 0)
    water_size = unreal.Vector(0.1, 0.1, 1)
    # Define the water area size
    water_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.WaterBodyOcean, water_location)
    water_actor.set_actor_label("Ocean")
    water_actor.set_actor_scale3d(water_size)
    unreal.log("Ocean created.")

def add_directional_light():
    location = unreal.Vector(0, 0, 100)  # Position in the scene
    rotation = unreal.Rotator(-45, 0, 0)  # Rotate to create sunlight effect
    directional_light = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.DirectionalLight, location)
    directional_light.set_actor_rotation(rotation, teleport_physics=True)
    directional_light.set_actor_label("DirectionalLight")
    unreal.log("Directional light added.")

def add_exponential_height_fog():
    location = unreal.Vector(0, 0, 100)
    fog = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.ExponentialHeightFog, location)
    fog.set_actor_label("ExponentialHeightFog")
    unreal.log("Exponential height fog added.")

def add_sky_atmosphere():
    location = unreal.Vector(0, 0, 100)
    sky_atmosphere = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.SkyAtmosphere, location)
    sky_atmosphere.set_actor_label("SkyAtmosphere")
    unreal.log("Sky atmosphere added.")

def add_skylight():
    location = unreal.Vector(0, 0, 100)
    skylight = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.SkyLight, location)
    skylight.set_actor_label("SkyLight")
    skylight.light_component.set_intensity(1.0) # Set brightness, adjust as needed
    unreal.log("Skylight added.")

def add_volumetric_cloud():
    location = unreal.Vector(0, 0, 100)
    volumetric_cloud = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.VolumetricCloud, location)
    volumetric_cloud.set_actor_label("VolumetricCloud")
    unreal.log("Volumetric cloud added.")

def generate_agent(name, pose):
    pos  = unreal.Vector(SCALE*pose[0],SCALE*pose[1],SCALE*pose[2])
    rot  = unreal.Rotator(0,0,pose[3])
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(ACTOR, pos, rot)
    actor.set_actor_label(f"{name}")
    unreal.log(f'Actor created: {actor.get_name()}')

if __name__=="__main__":

    # create client
    res = requests.get('http://127.0.0.1:5555/init_status')
    if res.status_code == 200:
        
        open_level()
        add_directional_light()
        add_sky_atmosphere()
        add_skylight()
        add_exponential_height_fog()
        add_volumetric_cloud()
        # load_ocean()

        # server request
        response = res.json()
        print(response['data'])
        agents = json.loads(response['data'])
        
        for name in agents.keys():
            pose = agents[name]
            generate_agent(name, pose)
            
        unreal.log('Init Completed.')