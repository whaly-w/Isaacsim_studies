from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import numpy as np
import carb 
import omni
import omni.appwindow
from isaacsim.core.api import World, SimulationContext
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.robots import Robot

from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy


# Functions
def _sub_keyboard_event(event, *args, **kwargs) -> bool:
    global _input_keyboard_mapping, _base_command
    """Subscriber callback to when kit is updated."""
    # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        # on pressing, the command is incremented
        if event.input.name in _input_keyboard_mapping:
            _base_command += np.array(_input_keyboard_mapping[event.input.name])

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        # on release, the command is decremented
        if event.input.name in _input_keyboard_mapping:
            _base_command -= np.array(_input_keyboard_mapping[event.input.name])
    return True

def _timeline_timer_callback_fn(self, event) -> None:
    global _physics_ready
    print('-------------------------------------------')
    if self.h1: _physics_ready = False


### Setups
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane(
    z_position=0,
    name="default_ground_plane",
    prim_path="/World/defaultGroundPlane",
    static_friction=0.2,
    dynamic_friction=0.2,
    restitution=0.01,)

sim_context = SimulationContext()
world.set_simulation_dt(physics_dt= 1.0 / 200, rendering_dt= 2.0 / 200)


# Global Variables
_base_command = [0.0, 0.0, 0.0]
_input_keyboard_mapping = {
    "W": [1, 0.0, 0.0],
    "D": [0.0, 0.0, 1],
    "A": [0.0, 0.0, -1]}
_physics_ready = False


# Add H1
assets_root_path = get_assets_root_path()
h1 = H1FlatTerrainPolicy(
    prim_path="/World/H1",
    name="H1",
    usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
    position=np.array([0, 0, 1.05]), )


# Timer Callback
timeline = omni.timeline.get_timeline_interface()
_event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
    int(omni.timeline.TimelineEventType.STOP), _timeline_timer_callback_fn)

### Post Load
_appwindow = omni.appwindow.get_default_app_window()
_input = carb.input.acquire_input_interface()
_keyboard = _appwindow.get_keyboard()
_sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, _sub_keyboard_event)



### Reset World
world.reset()




### Infinite Loop


# world.pause()
while True:
    # print(_base_command)
    world.step(render= True)
    if _physics_ready and not sim_context.is_stopped():
        h1.forward(sim_context.get_physics_dt(), _base_command)
    elif sim_context.is_stopped():
        _physics_ready = False
    else:
        _physics_ready = True
        h1.initialize()
        h1.post_reset()
        h1.robot.set_joints_default_state(h1.default_pos)
        

    

simulation_app.close()



