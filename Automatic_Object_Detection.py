"""Example of using the ObjectDetector class to detect objects in an image."""

from __future__ import print_function

import argparse
import glob
import logging
import os
import random
import math
import re
import sys
import weakref
import time

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# try:
#     import numpy as np
# except ImportError:
#     raise RuntimeError('cannot import numpy, make sure numpy pacakage is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
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


from Self_Driving_Agent import SelfDrivingAgent
import DetectingObject
from Controlling_Automatically import change_speed

from agents.navigation.behavior_agent import BehaviorAgent
# ==============================================================================
# -- Global Functions -----------------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# ==============================================================================
# -- World -----------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self.world = carla_world

        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_presets = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.hud = hud
        self.restart(args)
        

    def restart(self, args):
        """"Restart the world"""

        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

            # set the seed if requested by the user
        if args.seed is not None:
            random.seed(args.seed)

            # Get a random blueprint.
        #blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint = self.world.get_blueprint_library().find('vehicle.bmw.grandtourer')
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        #spawn the player
        print('spawning player')
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.spawn_actor(blueprint, spawn_point)

        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            spawn_point = spawn_points[57] if spawn_points else carla.Transform()
            self.player = self.world.spawn_actor(blueprint, spawn_point)

                # Set up the camera sensor
            self.camera_manager = CameraManager(self.player, self._gamma, args.width, args.height)
            self.camera_manager.transform_index = cam_pos_id
            self.camera_manager.set_sensor(1, notify=False)
            actor_type = get_actor_display_name(self.player)
            print('spawned %r at %s' % (actor_type, spawn_point.location))

    def next_weather(self, reverse=False):
        """Method to change the weather"""
        self._weather_presets += 1 if not reverse else -1
        self._weather_presets %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_presets]
        self.player.get_world().set_weather(preset[0])

    def render(self, display):
        """ Method to render the world """
        state, labels = self.camera_manager.render(display)
        self.hud.render(self, display)
        return state, labels

    def destroy_sensor(self):
        """Destroy the camera sensor"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None
                    
    def destroy(self):
        """Destroy the player"""
        actors = [
            self.camera_manager.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- HUD -----------------------------------------------------------------------------
# ==============================================================================

class HUD(object):
    """Class to manage the HUD"""

    def __init__(self, width, height):
        self.dim = (width, height)
        self._font = pygame.font.SysFont('Arial', 16)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)

    def render(self, world, display):
        """Method to render the HUD"""
        velocity = world.player.get_velocity()
        if velocity is not None:
            speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            display.blit(self._font.render('Speed: % 5d km/h' % speed, True, (255, 255, 255)), (10, 10))

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    def __init__(self, world):
        pass

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

class CameraManager(object):
    """ Class to manage the camera sensor """

    def __init__(self, parent_actor, gamma_correction, width, height):
        self.sensor = None
        self.labels = []
        self.state = False
        self.surface = None
        self._parent = parent_actor
        self._gamma = gamma_correction
        attachment = carla.AttachmentType
        self._camera_transforms = carla.Transform(carla.Location(x=1.6, z=1.7)), attachment.Rigid 
        bp_library = self._parent.get_world().get_blueprint_library()
        blp = bp_library.find('sensor.camera.rgb')
        blp.set_attribute('image_size_x', str(width))
        blp.set_attribute('image_size_y', str(height))
        blp.set_attribute('fov', '50')
        if blp.has_attribute('gamma'):
            blp.set_attribute('gamma', str(gamma_correction))
        self.sensor = self._parent.get_world().spawn_actor(blp, self._camera_transforms[0], attach_to=self._parent, attachment_type=self._camera_transforms[1]) 

    def set_sensor(self, index, notify=True):
        """Set the sensor"""
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        print("Camera sensor created")


    def render(self, display):
        """ Render method for the camera sensor """
        if self.surface is not None:
            display.blit(self.surface, (0, 0))
            if self.labels:
                return self.state, self.labels
            else: 
                return self.state, []

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if self is None:
            return
        self.surface, self.state, self.labels = DetectingObject.parse_image(image)


def game_loop(args):
    """ Main loop for the game """

    pygame.init()
    pygame.font.init() 
    world = None
    tot_target_reached = 0
    num_min_waypoints = 21
    tm_port = 9000
    desired_speed = 8.16

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        #world = World(client.load_world('TownTest01'), args)
        controller = KeyboardControl(world)
        status = True
        data = True
        data = False
        traffic_lights = world.world.get_actors().filter('traffic.traffic_light')
        # night = carla.WeatherParameters(
        #     sun_altitude_angle=-90.0  # Makes it fully night
        #     )

        #world.world.set_weather(night)

        for light in traffic_lights:
            light.set_state(carla.TrafficLightState.Green)
            light.freeze(True)  # This keeps it green permanently


# for the implementing the agent code
        spawn_points = world.map.get_spawn_points()
        agent = None
        if not(data):
            agent = SelfDrivingAgent(world.player)

            if spawn_points[0].location != agent.vehicle.get_location():
                destination = spawn_points[0].location
            else:
                destination = spawn_points[1].location
            destination = spawn_points[75].location
            agent.set_destination(agent.vehicle.get_location(), destination, clean=True)


        time.sleep(3)
        clock = pygame.time.Clock()
        print("Starting simulation")
        last_stop_time = time.time() - 86400
        state_time = time.time() - 86400

        vechicle_speed_state = time.time() - 86400

        while True:
            if controller.parse_events():
                return

            if not world.world.wait_for_tick(10.0):
                continue

            world.world.wait_for_tick(10.0)

            state, labels = world.render(display)
            pygame.display.flip()

            current_time = time.time()


            if state:
               state_time = time.time()
            

            some_state = False

#        Changing the desired speed based on Sign Detected
            if labels is None:
                pass
            elif current_time - state_time < 10.0 and "crosswalk-blue" in labels:
                some_state = True
                desired_speed = 0
            
            elif current_time - state_time < 2.0:
                some_state = True
                desired_speed = 0
                print ("Pedestrian detected, stopping vehicle");
            
            elif "crosswalk-red" in labels:
                desired_speed = 15/3.6
                vechicle_speed_state = time.time() - 25
            elif "crosswalk-blue"  in labels: 
                desired_speed = 10/3.6
                vechicle_speed_state = time.time() - 30
            elif "speed-30" in labels:
                desired_speed = 20/3.6
                vechicle_speed_state = time.time()
                print(labels)
            elif "speed-60" in labels:
                desired_speed = 30/3.6
                vechicle_speed_state = time.time()
                print(labels)
            elif current_time - vechicle_speed_state >= 40:
                desired_speed = 8.16

# Using autopilot code
            if (data):
                world.player.set_autopilot(True, tm_port)
                last_stop_time = change_speed(client.get_world(),some_state, world.map, world.player, desired_speed, last_stop_time)

# Using Behaviour agent
            else:
                agent.update_information(world, desired_speed*3.6)

                if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints and args.loop:
                    agent.reroute(spawn_points)
                    tot_target_reached += 1
                    print("No more waypoints, ending simulation")

                elif len(agent.get_local_planner().waypoints_queue) == 0 and not args.loop:
                    print("Target reached, stopping simulation")
                    break

                speed_limit = world.player.get_speed_limit()
                agent.get_local_planner().set_speed(speed_limit)
                control = agent.run_step()
                world.player.apply_control(control)
                


    finally:
        if world is not None:
            world.destroy()

        pygame.quit()
        

# ==============================================================================
# -- main() -----------------------------------------------------------------
# ==============================================================================

def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument('-d',
        '--data',
        type=bool,
        default=False,
        help='Use custom code')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()

