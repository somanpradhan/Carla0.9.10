import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import cv2
import numpy as np
import pandas as pd
import time
from ultralytics import YOLO

# === GLOBALS ===
#model = YOLO('models/best.pt')

model = YOLO("G:\\Training\\Training\\runs\\detect\\train8\\weights\\best.pt").to("cuda")  # Load the YOLOv8 model (Replace with your trained model path)
log_data = []

weather_presets = [
    carla.WeatherParameters.ClearNoon,
    carla.WeatherParameters.CloudyNoon,
    carla.WeatherParameters.WetNoon,
    carla.WeatherParameters.MidRainyNoon,
    carla.WeatherParameters.HardRainNoon,
    carla.WeatherParameters.SoftRainSunset
]

# Optional: map to friendly names for logs
weather_names = {
    carla.WeatherParameters.ClearNoon: "ClearNoon",
    carla.WeatherParameters.CloudyNoon: "CloudyNoon",
    carla.WeatherParameters.WetNoon: "WetNoon",
    carla.WeatherParameters.MidRainyNoon: "MidRainyNoon",
    carla.WeatherParameters.HardRainNoon: "HardRainNoon",
    carla.WeatherParameters.SoftRainSunset: "SoftRainSunset"
}

# === HELPERS ===
def get_speed(vehicle):
    v = vehicle.get_velocity()
    speed = (v.x**2 + v.y**2 + v.z**2)**0.5
    return speed * 3.6  # m/s to km/h

def store_log(timestamp, class_name, conf, speed, weather_name):
    log_data.append({
        'timestamp': timestamp,
        'class': class_name,
        'confidence': conf,
        'vehicle_speed': speed,
        'weather': weather_name
    })

def control_vehicle(vehicle, detections):
    control = carla.VehicleControl()
    speed_limit = 50 / 3.6  # default limit in m/s

    if 'red_sign' in detections:
        speed_limit = 30 / 3.6

    if 'blue_sign' in detections:
        if 'pedestrian' in detections:
            control.throttle = 0.0
            control.brake = 1.0
        else:
            control.throttle = 0.2
            control.brake = 0.0
    else:
        current_speed = get_speed(vehicle) / 3.6
        if current_speed < speed_limit:
            control.throttle = 0.4
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = 0.2

    vehicle.apply_control(control)

def process_image(image, vehicle, weather_name):
    img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    img_array = img_array.reshape((image.height, image.width, 4))
    img_bgr = img_array[:, :, :3]

    results = model(img_bgr)
    current_detections = set()

    for det in results[0].boxes.data.tolist():
        x1, y1, x2, y2, conf, cls = det
        class_name = model.names[int(cls)]
        current_detections.add(class_name)

        store_log(time.time(), class_name, conf, get_speed(vehicle), weather_name)

    control_vehicle(vehicle, current_detections)

    cv2.imshow("Camera", img_bgr)
    cv2.waitKey(1)

def spawn_vehicle(world, blueprint_library):
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is not None:
            print(f"Spawned vehicle at {spawn_point.location}")
            return vehicle
    raise RuntimeError("Failed to spawn vehicle at any spawn point")

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Clean up existing actors
        actors = list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('sensor.*'))
        for actor in actors:
            actor.destroy()

        vehicle = spawn_vehicle(world, blueprint_library)

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        for weather in weather_presets:
            weather_name = weather_names.get(weather, str(weather))
            print(f"\n=== Running scenario in weather: {weather_name} ===")
            world.set_weather(weather)
            time.sleep(2)

            camera.listen(lambda image: process_image(image, vehicle, weather_name))

            time.sleep(30)
            camera.stop()

        df = pd.DataFrame(log_data)
        df.to_csv("logs/detection_log.csv", index=False)
        print("Log saved to logs/detection_log.csv")

    except Exception as e:
        print(f"Error occurred: {e}")

    finally:
        print("Cleaning up actors...")
        actors = list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('sensor.*'))
        for actor in actors:
            actor.destroy()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()