# Objectives
# 1) Import object using RigidPrim
# 2) Timestamps

# launch Isaac Sim before any other imports -> default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
import omni
import numpy as np
from pxr import UsdLux

from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.prims import RigidPrim
from isaacsim.core.api.objects import DynamicCuboid


### Setups
world = World()
stage = omni.usd.get_context().get_stage()

world.scene.add(GroundPlane(prim_path="/World/groundPlane", size=50, color=np.array([0.5, 0.5, 0.5])))
distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")


# create three rigid cubes sitting on top of three others
for i in range(3):
    DynamicCuboid(prim_path=f"/World/bottom_box_{i+1}", size=2, color=np.array([0.5, 0, 0]), mass=1.0)
    DynamicCuboid(prim_path=f"/World/top_box_{i+1}", size=2, color=np.array([0, 0, 0.5]), mass=1.0)

# as before, create RigidContactView to manipulate bottom boxes but this time specify top boxes as filters to the view object
# this allows receiving contact forces between the bottom boxes and top boxes
bottom_box = RigidPrim(
    prim_paths_expr= "/World/bottom_box_*",
    name= "bottom_box",
    positions= np.array([[0, 0, 1.0], [-5.0, 0, 1.0], [5.0, 0, 1.0]]),
    contact_filter_prim_paths_expr= ["/World/top_box_*"])

# create a RigidContactView to manipulate top boxes
top_box = RigidPrim(
    prim_paths_expr= "/World/top_box_*",
    name= "top_box",
    positions= np.array([[0.0, 0, 3.0], [-5.0, 0, 3.0], [5.0, 0, 3.0]]),
    track_contact_forces=True)

world.scene.add(top_box)
world.scene.add(bottom_box)


### Reset World
world.reset()

# Tag world time
timeline = omni.timeline.getimeline = omni.timeline.get_timeline_interface()


### Infinite Loop
_ = input('Press Enter to Start...')
while True:
    timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
    print(f'----------------------< {round(timecode)} >----------------------')
    
    # print(bottom_box.get_net_contact_forces())
    print(bottom_box.get_contact_force_matrix())
    
    world.step(render=True)
    
simulation_app.close() 