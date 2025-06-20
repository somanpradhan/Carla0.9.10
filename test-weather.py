import glob
import os
import sys

try:
    sys.path.append(glob.glob('C:\\Users\\Shrijan\\Desktop\\carla\\Build\\UE4Carla\\0.9.10-dirty\\WindowsNoEditor\\PythonAPI\\carla\\dist\\carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import csv

def setup_vehicle(world):
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.*model3")[0]  # or any other vehicle
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle

def cleanup(vehicle):
    if vehicle.is_alive:
        vehicle.destroy()

def run_speed_limit_scenario(world, vehicle, log_writer):
    """
    Dummy example – replace with your actual logic.
    You can log vehicle speed, acceleration, throttle, distance to sign, etc.
    """
    for i in range(20):  # Simulate 20 steps
        velocity = vehicle.get_velocity()
        speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # km/h
        log_writer.writerow([world.get_weather(), round(speed, 2)])
        time.sleep(0.2)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    weather_presets = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("WetSunset", carla.WeatherParameters.WetSunset),
    ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
    ("MidRainSunset", carla.WeatherParameters.MidRainSunset),
    ("HardRainSunset", carla.WeatherParameters.HardRainSunset),
    ("MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
    ]


    with open("speed_test_results.csv", mode="w", newline="") as f:
        log_writer = csv.writer(f)
        log_writer.writerow(["Weather", "VehicleSpeed(km/h)"])

        for name, weather in weather_presets:
            print(f"\n=== Running scenario under: {name} ===")
            world.set_weather(weather)
            time.sleep(1.0)

            vehicle = setup_vehicle(world)
            time.sleep(2.0)

            try:
                run_speed_limit_scenario(world, vehicle, log_writer)
            finally:
                cleanup(vehicle)
                time.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Test interrupted.")
