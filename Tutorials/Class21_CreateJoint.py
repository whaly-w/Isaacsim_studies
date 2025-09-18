# Objectives
# 1) create & control joints in python standalone

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
import omni
import numpy as np
from pxr import UsdLux, UsdPhysics, Gf

from isaacsim.core.prims import SingleRigidPrim, RigidPrim, SingleArticulation
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.objects import DynamicCuboid


# -------------- Setups --------------
world = World()
stage = world.stage
distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
distant_light.CreateIntensityAttr(1000)

world.scene.add_default_ground_plane()

# Create three dynamic cubes
cube_prim = []
color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1]]
for i in range(5):
    cube_prim.append(f"/World/random_cube_0{i}")
    DynamicCuboid(
        prim_path= cube_prim[i], 
        size= 1, 
        # color= np.random.uniform(0, 1, 3), 
        color= np.array(color[i]), 
        position= np.array([0, i*3, 1]))
    

# -------------- Global variables --------------
world_frame = '/World'



# -------------- Add fixed joint to box01 --------------
box01_joint_path = cube_prim[1] + '/fixed_joint'
box01_joint = UsdPhysics.FixedJoint.Define(stage, box01_joint_path)

# set target frame
box01_joint.CreateBody0Rel().SetTargets([world_frame])
box01_joint.CreateBody1Rel().SetTargets([cube_prim[1]])

# set attribute of fixedJoint, relative to body0
# if not set, it will be the object defualt position & orientation
box01_joint.CreateLocalPos0Attr().Set((0, 3, 1))
box01_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))



# -------------- Add revolute joint to box02 --------------
box02_Xjoint_path = cube_prim[2] + '/revolute_joint'
box02_joint = UsdPhysics.RevoluteJoint.Define(stage, box02_Xjoint_path)

# set target joint and rotational axis
box02_joint.CreateBody0Rel().SetTargets([world_frame])
box02_joint.CreateBody1Rel().SetTargets([cube_prim[2]])
box02_joint.CreateAxisAttr().Set("Z")

# apply DriveAPI to control movement of joint: angular or linear
# Apply(prim, name)
box02_drive = UsdPhysics.DriveAPI.Apply(box02_joint.GetPrim(), "angular")
# set properities
box02_drive.CreateTargetPositionAttr().Set(30)
box02_drive.CreateStiffnessAttr().Set(100)
box02_drive.CreateDampingAttr().Set(10.0)
box02_drive.CreateMaxForceAttr().Set(1000.0)



# -------------- Add prismatic joint to box03 --------------
box03_joint_path = cube_prim[3] + '/prismatic_joint'
box03_joint = UsdPhysics.PrismaticJoint.Define(stage, box03_joint_path)

# set target joint and rotational axis
box03_joint.CreateBody0Rel().SetTargets([world_frame])
box03_joint.CreateBody1Rel().SetTargets([cube_prim[3]])
box03_joint.CreateAxisAttr().Set("X")

# set revolute joint properities
box03_drive = UsdPhysics.DriveAPI.Apply(box03_joint.GetPrim(), "linear")
box03_drive.CreateTargetPositionAttr().Set(0)
box03_drive.CreateStiffnessAttr().Set(100)
box03_drive.CreateDampingAttr().Set(10.0)
box03_drive.CreateMaxForceAttr().Set(1000.0)




# -------------- Add spherical joint to box04 --------------
box04_joint_path = cube_prim[4] + '/spherical_joint'
box04_joint = UsdPhysics.SphericalJoint.Define(stage, box04_joint_path)

# set target joint and rotational axis
box04_joint.CreateBody0Rel().SetTargets([world_frame])
box04_joint.CreateBody1Rel().SetTargets([cube_prim[4]])

# spherical joint have no DriveAPI supports
# so, a wrapper is created to control box04
box04 = SingleRigidPrim(prim_path= cube_prim[4], name= 'box04')




# -------------- Infinite Loop--------------
world.reset()
_dir = 1
while True:
    world.step(render=True)
    _time = world.current_time
    if _time % 10 > 5:
        _dir = -1
    else:
        _dir = 1
    
    # rotate box02
    box02_drive.CreateTargetPositionAttr().Set((_time * 30) % 360)
    
    # move box03
    box03_drive.CreateTargetPositionAttr().Set(((_time * 1) % 5) * _dir)
    
    # rotate box04
    box04.set_angular_velocity(np.array([10 * _dir, 10 * _dir, 0]))
    
simulation_app.close() 

