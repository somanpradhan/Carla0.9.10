import glob
import os
import sys
import random
import argparse
import logging

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


import re
import numpy as np
import cv2
import time
from ultralytics import YOLO
from Lane_Detection import process_image_lane
import math
import weakref


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
        self.camera_manager.render(display)
        self.hud.render(self, display)

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
        self.surface = None
        self._parent = parent_actor
        self._gamma = gamma_correction
        attachment = carla.AttachmentType
        # self._camera_transforms = carla.Transform(carla.Location(x=1.6, z=1.7)), attachment.Rigid 
        self._camera_transforms = carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), attachment.SpringArm
        bp_library = self._parent.get_world().get_blueprint_library()
        blp = bp_library.find('sensor.camera.rgb')
        blp.set_attribute('image_size_x', str(width))
        blp.set_attribute('image_size_y', str(height))
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

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if self is None:
            return

        self.surface = process_image_lane(image)

def game_loop(args):
    """ Main loop for the game """

    pygame.init()
    pygame.font.init() 
    world = None


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

# for the implementing the agent code
        time.sleep(3)
        clock = pygame.time.Clock()
        print("Starting simulation")


        while True:
            if controller.parse_events():
                return

            if not world.world.wait_for_tick(10.0):
                continue

            world.world.wait_for_tick(10.0)

            world.render(display)
            pygame.display.flip()
            world.player.set_autopilot(True)
            yaw =  world.player.get_transform().rotation.yaw
            # Convert degrees to radians
            yaw_rad = math.radians(yaw)

            forward_vector = carla.Vector3D(
                math.cos(yaw_rad),
                math.sin(yaw_rad),
                0
            )
            print(yaw)



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

