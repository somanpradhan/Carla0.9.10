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
import math
import time

# === Route configuration ===
#ROUTE_IDS = [125, 193, 100, 102, 182, 235, 208, 145, 105, 103, 181, 101, 99, 192, 124]
#ROUTE_IDS = [210, 112, 114, 163,206, 178, 172, 168, 195, 158,26,16,14,12,10,260,58,56,128,50,134,95,73,140,52,144,199]
#ROUTE_IDS = [210, 112, 114, 163,206, 178, 172, 168, 195, 158,26,16,14,12,10,260,58,56,128]
#ROUTE_IDS = [ 34, 33, 32, 157, 68, 69] 1
#ROUTE_IDS = [ 243, 250, 1, 147, 20] 2
ROUTE_IDS = [190, 98, 257, 8, 229, 232, 230, 212, 42, 40, 38, 165, 67, 36, 35, 159, 27, 25, 15, 13, 11, 261, 59]

WAYPOINT_THRESHOLD = 2.0  # meters

def get_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)

def move_to_target(vehicle, target, speed=10.0):
    control = carla.VehicleControl()
    vehicle_loc = vehicle.get_location()
    vehicle_yaw = math.radians(vehicle.get_transform().rotation.yaw)

    # Direction vector to target
    dx = target.x - vehicle_loc.x
    dy = target.y - vehicle_loc.y
    target_angle = math.atan2(dy, dx)

    # Steering angle
    angle_diff = target_angle - vehicle_yaw
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize

    control.throttle = 0.5
    control.steer = max(-1.0, min(1.0, angle_diff))
    control.brake = 0.0
    vehicle.apply_control(control)

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()
    spawn_points = map.get_spawn_points()

    # Validate spawn ids
    if any(i >= len(spawn_points) for i in ROUTE_IDS):
        print("Invalid spawn ID")
        return

    # Get list of carla.Locations
    route_locations = [spawn_points[i].location for i in ROUTE_IDS]

    # Spawn the vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.*model3*")[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[ROUTE_IDS[0]])

    # Set spectator to follow
    spectator = world.get_spectator()
    def follow_spectator():
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50), carla.Rotation(pitch=-90)))

    try:
        idx = 1
        while idx < len(route_locations):
            target = route_locations[idx]
            move_to_target(vehicle, target)
            follow_spectator()

            if get_distance(vehicle.get_location(), target) < WAYPOINT_THRESHOLD:
                idx += 1
                print(f"Reached waypoint {idx}/{len(route_locations)}  {ROUTE_IDS[idx]} (index {idx})")
            time.sleep(0.05)

        print("✅ Finished route.")
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

    except KeyboardInterrupt:
        print("🛑 Interrupted.")
    finally:
        vehicle.destroy()

if __name__ == "__main__":
    main()
