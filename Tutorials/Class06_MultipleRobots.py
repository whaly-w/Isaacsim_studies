# Objectives
# 1) Introduce concept of multiple robot in a simulation
# 2) Create custom task
# 3) Create logic control in task


from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.manipulators.examples.franka.tasks import PickPlace
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.utils.types import ArticulationAction




# Define custom task
class RobotsPlaying(BaseTask):
    def __init__(self, name):
        super().__init__(name= name, offset=None)
        
        # Set target positoin for jetbot
        self._jetbot_goal_position = np.array([1.3, 0.3, 0])

        # Create franka + cube
        self._pick_place_task = PickPlace(cube_initial_position=np.array([0.1, 0.3, 0.05]),
                                        target_position=np.array([0.7, -0.3, 0.0515 / 2.0]))
        
        # Task Logic
        self._task_event = 0
        
        return


    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        
        # Add Jetbot
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self._jetbot = scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=np.array([0, 0.3, 0]))
        )

        # Tag Franka
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])

        # Changes Franka's default position
        # so that it is set at this position after reset
        self._franka.set_world_pose(position=np.array([1.0, 0, 0]))
        self._franka.set_default_state(position=np.array([1.0, 0, 0]))

        return


    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()

        # Add jetbot properties and task_event to observation
        observations= {
            'task_event': self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        
        # Add franka properties to observation
        observations.update(self._pick_place_task.get_observations())

        return observations


    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value": self._jetbot.name, "modifiable": False}
        params_representation["franka_name"] = pick_place_params["robot_name"]

        return params_representation


    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 0.04:
                self._task_event += 1
                self._cube_arrive_step_index = control_index

        elif self._task_event == 1:
            # Jetbot has 200 time steps to back off from Franka
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1

        return


    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._task_event = 0
        return



### Setups
world = World()
world.scene.add_default_ground_plane()

# Create task
world.add_task(RobotsPlaying(name='super_cool_task'))


### Reset world
world.reset()

task_params = world.get_task('super_cool_task').get_params()

cube_name = task_params['cube_name']['value']

jetbot = world.scene.get_object(task_params['jetbot_name']['value'])
jetbot_controller = WheelBasePoseController(name="cool_controller",
                                            open_loop_wheel_controller= DifferentialController(name="simple_control",
                                                                                               wheel_radius=0.03, 
                                                                                               wheel_base=0.1125))

franka = world.scene.get_object(task_params['franka_name']['value'])
franka_controller = PickPlaceController(
    name= 'pick_place_controller',
    gripper= franka.gripper,
    robot_articulation= franka
)

jetbot_controller.reset()
franka_controller.reset()


# Infinite Loop
while True:
    
    current_observations = world.get_observations()
    
    # At event 0 -> jetbot diliver the box
    if current_observations['task_event'] == 0:
        jetbot.apply_action(jetbot_controller.forward(
            start_position=current_observations[jetbot.name]["position"],
            start_orientation=current_observations[jetbot.name]["orientation"],
            goal_position=current_observations[jetbot.name]["goal_position"]))
        
    # At event 1 -> jetbot back off 200 steps
    elif current_observations['task_event'] == 1:
        jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[-8, -8]))
        
    # At event 2 -> jetbot stop
    elif current_observations['task_event'] == 2:
        jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[0, 0]))
        
        actions = franka_controller.forward(
            picking_position= current_observations[cube_name]['position'],
            placing_position= current_observations[cube_name]['target_position'],
            current_joint_positions= current_observations[franka.name]['joint_positions']
            )
        franka.apply_action(actions)
        
    
    if franka_controller.is_done():
        world.pause()
    
    world.step(render= True)
    
    

simulation_app.close()



