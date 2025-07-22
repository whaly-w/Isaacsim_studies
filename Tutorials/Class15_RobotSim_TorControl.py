# Objectives
# 1) Create torque controller
# 2) articulatino query

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
frankas_view = Articulation(prim_paths_expr="/World/Franka_[1-2]", name="frankas_view")
world.scene.add(frankas_view)

# Set franka position
new_positions = np.array([[-1.0, 1.0, 0], [1.0, 1.0, 0]])
frankas_view.set_world_poses(positions=new_positions)

### Reset World
world.reset()

# Setup controller
dc = _dynamic_control.acquire_dynamic_control_interface()
articulation_1 = dc.get_articulation("/World/Franka_1")
articulation_2 = dc.get_articulation('/World/Franka_2')

dc.wake_up_articulation(articulation_1)
dc.wake_up_articulation(articulation_2)

# Check Object Type
obj_type = dc.peek_object_type('/World/Franka_1')
print(f'-----------------------> {obj_type}')

# Query articuation
num_joints = dc.get_articulation_joint_count(articulation_2)
num_dofs = dc.get_articulation_dof_count(articulation_2)
num_bodies = dc.get_articulation_body_count(articulation_2)
print(f'-----------------------> n_joint: {num_joints}, n_dof: {num_dofs}, n_body: {num_bodies}')

# Select one DOF
dof_ptr = dc.find_articulation_dof(articulation_2, "panda_joint1")



### Infinite Loop
world.pause()
while True:
    dc.wake_up_articulation(articulation_1)
    dc.wake_up_articulation(articulation_2)

    # Torque Control -> Give robot random Joint torque
    joint_efforts = [-np.random.rand(9) * 1000]
    dc.set_articulation_dof_efforts(articulation_1, joint_efforts)
    dc.set_articulation_dof_efforts(articulation_2, [np.array([-500, -500, -500, -500, -500, -np.random.rand(1)[0] * 1000, -500, -500, -500])])
    
    # this will show the state of each DOF [pos, vel, effort]
    dof_state_list = dc.get_articulation_dof_states(articulation_1, _dynamic_control.STATE_ALL)
    print(f'Franka_1 DOF=>\n{dof_state_list}')
    
    dof_state = dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)
    print(f'Franka_2 Joint1 Torque=> {dof_state.effort}')
    

    world.step(render=True)
    
simulation_app.close() 
