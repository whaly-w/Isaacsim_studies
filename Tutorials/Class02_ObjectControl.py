# Objectives
# 1) Using command method in XFormPrim


# launch Isaac Sim 
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

import numpy as np

# Create a world + setups
world = World()
world.scene.add_default_ground_plane()
fancy_cube =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 0.25]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0, 0, 1.0]),
    ))


# Reset environments
world.reset()

# Start infinite loop
while True:
    world.step(render=True) 
    
    # XFormPrim also have command methods: set_linear_velocity, set_angular_velocity, 
    fancy_cube.set_angular_velocity(np.array([0, 0, 150]))
    fancy_cube.set_linear_velocity(np.array([0.5, 0, 0]))
    
    print('Random_cube states:')
    print(f'> Linear Vel:\t{fancy_cube.get_linear_velocity()}')
    print(f'> Angular Vel:\t{fancy_cube.get_angular_velocity()}')

    
# close Isaac Sim
simulation_app.close() 
