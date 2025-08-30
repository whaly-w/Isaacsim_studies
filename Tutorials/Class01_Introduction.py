# Objectives
# 1) Create world + Plane + Object
# 2) Track object properties
# 3) Run custom script


# launch Isaac Sim before any other imports -> default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np

# Create a world + setups
world = World()
world.scene.add_default_ground_plane()

# world.scene.add() returns an object associated with the input argument 
# which for DynamicCuboid would be an isaacsim.core.prim.XFormPrim
fancy_cube =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 1.5]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    ))

# Reset environments & initialize physcis
world.reset()

# Start infinite loop
while True:
    # Using methods in XFormPrim class to get object states (https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.prims/docs/index.html#isaacsim.core.prims.SingleXFormPrim)
    position, orientation = fancy_cube.get_world_pose()
    linear_velocity = fancy_cube.get_linear_velocity()
    
    # will be shown on terminal
    print("<------------------------- update --------------------------->")
    print("Cube position is : " + str(position))
    print("Cube's orientation is : " + str(orientation))
    print("Cube's linear velocity is : " + str(linear_velocity))
    
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step
    
# close Isaac Sim
simulation_app.close() 

# To run this script -> activate env and run it normally -> "python ~/isaac_ws/Tutorials/Class01_Introduction.py"