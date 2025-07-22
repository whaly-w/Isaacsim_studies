# Objectives
# 1) Setup scene without using default build-in

# launch Isaac Sim before any other imports -> default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.core.api.objects.ground_plane import GroundPlane
import omni.usd
from pxr import UsdLux, Gf, UsdGeom

### Setups
world = World()
world.scene.add(GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5])))
world.scene.add(DynamicCuboid(prim_path="/World/cube",
                              position= np.array([-.5, -.2, 1.0]),
                              scale= np.array([.5, .5, .5]),
                              color= np.array([.2,.3,0.])))

# Add light source
# Get the current stage
stage = omni.usd.get_context().get_stage()

# Create a distant light
distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
distant_light.CreateIntensityAttr(5000)  # Brightness (default: 500)
distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))  # RGB (white)
distant_light.CreateAngleAttr(0.53)  # Angular size (softens shadows)

# Optional: Rotate the light to change direction
light_prim = distant_light.GetPrim()
xform = UsdGeom.Xformable(light_prim)
xform.AddRotateYOp().Set(45)  # 45° around Y-axis


### Reset World
world.reset()


### Infinite Loop
while True:
    world.step(render=True)
    
simulation_app.close() 