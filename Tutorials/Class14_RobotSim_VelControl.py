# Objectives
# 1) Create velocity controller

# launch Isaac Sim before any other imports -> default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
import omni
import numpy as np
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage

from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdPhysics


### Setups
world = World()
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane()

# Add Franka
asset_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
for i in range(1, 3):
    add_reference_to_stage(usd_path=asset_path, prim_path=f"/World/Franka_{i}")
frankas_view = Articulation(prim_paths_expr="/World/Franka_[1-3]", name="frankas_view")
world.scene.add(frankas_view)


### Reset World
world.reset()

# Set each franka position
new_positions = np.array([[-1.0, 1.0, 0], [1.0, 1.0, 0]])
frankas_view.set_world_poses(positions=new_positions)
frankas_view.set_joint_positions(np.array([[1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5],
                                           [0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]]))

# Set Stiffness of Joints
for prim in stage.TraverseAll():
    prim_type = prim.GetTypeName()
    drive = None
    if prim_type == 'PhysicsRevoluteJoint':
        drive = UsdPhysics.DriveAPI.Get(prim, 'angular')
    elif prim_type == 'PhysicsPrismaticJoint':
        drive = UsdPhysics.DriveAPI.Get(prim, 'linear')
    if drive is not None: 
        drive.GetStiffnessAttr().Set(0)
        
# Set Stiffness of a certain joint
panda_joint1_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/Franka_2/panda_link0/panda_joint1"), "angular")
panda_joint1_drive.GetStiffnessAttr().Set(0)


# Setup controller
dc = _dynamic_control.acquire_dynamic_control_interface()
articulation_1 = dc.get_articulation("/World/Franka_1")
articulation_2 = dc.get_articulation('/World/Franka_2')

dc.wake_up_articulation(articulation_1)
dc.wake_up_articulation(articulation_2)

# Select one DOF
dof_ptr = dc.find_articulation_dof(articulation_2, "panda_joint1")



### Infinite Loop
while True:
    # Velocity Control -> Give robot random Joint velocity
    joint_vels = [-np.random.rand(9) * 10]
    dc.set_articulation_dof_velocity_targets(articulation_1, joint_vels)
    
    dc.set_dof_velocity_target(dof_ptr, 0.2)
    
    
    world.step(render=True)
    
simulation_app.close() 
