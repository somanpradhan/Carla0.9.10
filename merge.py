import glob
import os
import sys
import math
import random
import numpy as np
import pygame
import time
import argparse
import signal

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ==================== Utility ====================
def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))

def get_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)

def move_to_target(vehicle, target, speed=10.0):
    control = carla.VehicleControl()
    vehicle_loc = vehicle.get_location()
    vehicle_yaw = math.radians(vehicle.get_transform().rotation.yaw)

    dx = target.x - vehicle_loc.x
    dy = target.y - vehicle_loc.y
    target_angle = math.atan2(dy, dx)

    angle_diff = target_angle - vehicle_yaw
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

    control.throttle = 0.5
    control.steer = max(-1.0, min(1.0, angle_diff))
    control.brake = 0.0
    vehicle.apply_control(control)

# ==================== Environment ====================
class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)

class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False

    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)

class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self):
        return '%s %s' % (self._sun, self._storm)

# ==================== Display and Detection ====================
WIDTH, HEIGHT = 640, 480
pygame.init()
display = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 18)

def show_camera_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    rgb_image = array[:, :, :3][:, :, ::-1]
    surface = pygame.surfarray.make_surface(rgb_image.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    return rgb_image

def detect_pedestrian_rgb(rgb_array):
    roi = rgb_array[HEIGHT // 2 :, :]
    red_channel = roi[:, :, 0]
    green_channel = roi[:, :, 1]
    blue_channel = roi[:, :, 2]
    mask = (red_channel > 120) & (green_channel < 100) & (blue_channel < 100)
    return np.sum(mask) > 500

# ==================== Route ====================
ROUTE_IDS = [190, 98, 257, 8, 229, 232, 230, 212, 42, 40, 38, 165, 67, 36, 35, 159, 27, 25, 15, 13, 11, 261, 59]
WAYPOINT_THRESHOLD = 2.0

# ==================== Main ====================
def main():
    actor_list = []
    rgb_image = None

    def signal_handler(sig, frame):
        print('\nInterrupted! Cleaning up and exiting.')
        for actor in actor_list:
            if actor is not None:
                actor.destroy()
        pygame.quit()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    weather = Weather(world.get_weather())
    map = world.get_map()
    spawn_points = map.get_spawn_points()

    if any(i >= len(spawn_points) for i in ROUTE_IDS):
        print("Invalid spawn ID")
        return

    route_locations = [spawn_points[i].location for i in ROUTE_IDS]
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.*model3*")[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[ROUTE_IDS[0]])
    actor_list.append(vehicle)

    # Camera setup
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(WIDTH))
    camera_bp.set_attribute("image_size_y", str(HEIGHT))
    camera_bp.set_attribute("fov", "90")
    camera_transform = carla.Transform(carla.Location(x=-6.0, z=3.0), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)

    def camera_callback(image):
        nonlocal rgb_image
        rgb_image = show_camera_image(image)

    camera.listen(camera_callback)

    for light in world.get_actors().filter('traffic.traffic_light*'):
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)

    spectator = world.get_spectator()
    def follow_spectator():
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50), carla.Rotation(pitch=-90)))

    idx = 1
    elapsed_time = 0.0
    speed_factor = 1.0
    update_freq = 0.1 / speed_factor

    try:
        while True:
            timestamp = world.wait_for_tick(seconds=30.0).timestamp
            elapsed_time += timestamp.delta_seconds
            if elapsed_time > update_freq:
                weather.tick(speed_factor * elapsed_time)
                world.set_weather(weather.weather)
                elapsed_time = 0.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            if rgb_image is not None:
                velocity = vehicle.get_velocity()
                speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

                status = "Moving" if speed > 0.1 else "Stopped"
                follow_spectator()

                if idx < len(route_locations):
                    move_to_target(vehicle, route_locations[idx])
                    if get_distance(vehicle.get_location(), route_locations[idx]) < WAYPOINT_THRESHOLD:
                        idx += 1
                        print(f"Reached waypoint {idx}/{len(route_locations)}")

                if detect_pedestrian_rgb(rgb_image):
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

                lines = [
                    f"Speed: {speed:.2f} km/h",
                    f"Status: {status}",
                    str(weather)
                ]
                for i, line in enumerate(lines):
                    text_surface = font.render(line, True, (255, 255, 255))
                    display.blit(text_surface, (10, 10 + i * 20))

                pygame.display.flip()

            clock.tick(30)

    finally:
        print("\nCleaning up actors...")
        for actor in actor_list:
            if actor:
                actor.destroy()
        pygame.quit()

if __name__ == '__main__':
    main()