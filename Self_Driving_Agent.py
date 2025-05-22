import os
import sys 
import time
import glob

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode -------------------------------------------
# ==============================================================================

try:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/carla')
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla

from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.local_planner_behavior import LocalPlanner, RoadOption
from agents.navigation.types_behavior import Cautious, Aggressive, Normal
from agents.navigation.basic_agent import BasicAgent

from agents.tools.misc import get_speed

class SelfDrivingAgent(BehaviorAgent):
    """
    Self-driving agent that uses Carla's BehaviorAgent to navigate through the environment.
    """

    def __init__(self, vehicle,ignore_traffic_light=False, behavior_type='normal'):
        """
        Initialize the SelfDrivingAgent.

        :param vehicle: The vehicle controlled by the agent.
        :param behavior_type: The type of behavior for the agent (cautious, normal, aggressive).
        :param target_speed: The target speed for the agent in km/h.
        """
        super(SelfDrivingAgent, self).__init__(vehicle)
        self.vehicle = vehicle
        self.ignore_traffic_light = ignore_traffic_light
        self._local_planner = LocalPlanner(self)
        self._grp = None
        self.look_ahead_steps = 0


        # Vechiles Information
        self.speed = 0
        self.speed_limit = 0
        self.direction = None
        self.incoming_direction = None
        self.start_waypoint = None
        self.end_waypoint = None
        self.incoming_waypoint = None
        self.start_waypoint = None
        self.end_waypoint = None
        self._ist_at_traffic_light = 0
        self.light_state = "Green"
        self.light_id_to_ignore = -1
        self.min_speed = 5
        self.behavior = None
        self._sampling_resolution = 4.5
        

        # Set up behavior type
        if behavior_type == 'cautious':
            self.behavior = Cautious()

        elif behavior_type == 'aggressive':
            self.behavior = Aggressive()

        else:
            self.behavior = Normal()

        # timer for stop sign
        self.stop_timer = time.time()
        self.state = True
        self.first_state = True


    def update_information(self, world, speed_limit=None):
        """
        Update the agent's information based on the current world state.

        :param world: The Carla world object.
        :param speed_limit: The speed limit for the agent in km/h.
        """
        self.speed = get_speed(self.vehicle)
        if speed_limit is not None:
            self.speed_limit = speed_limit
        else:
            self.speed_limit = self.vehicle.get_speed_limit()
        self._local_planner.set_speed(self.speed_limit)
        self.direction = self._local_planner.target_road_option
        if self.direction is None:
            self.direction = RoadOption.LANEFOLLOW
        
        self.look_ahead_steps = int ((self.speed_limit) / 10)

        self.incoming_waypoint, self.incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self.look_ahead_steps)
        self.is_at_traffic_light = world.player.is_at_traffic_light()
        current_time = time.time()
        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            #This method also includes stop sign and intersections.
            new_state = str(self.vehicle.get_traffic_light_state())

            if new_state == "Green":
                self.light_state = "Green"
            if  not(self.is_at_traffic_light) and new_state == "Red" and self.state == False and (current_time - self.stop_timer > 10):
                self.state = True
                self.light_state = "Green"
                self.stop_timer = time.time()
            elif not(self.is_at_traffic_light) and new_state == "Red" and self.state and (self.first_state or current_time - self.stop_timer > 10) :
                self.first_state = False
                self.state = False
                self.stop_timer = time.time()
                self.light_state = "Red"
            elif self.is_at_traffic_light:
                self.light_state = new_state

        



class CustomAgent(BasicAgent):
    def __init__(self, vehicle, target_speed=20, debug=False):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param target_speed: speed (in Km/h) at which the vehicle will move
        """
        super().__init__(target_speed, debug)

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """
        # Actions to take during each simulation step
        control = carla.VehicleControl()
        return control