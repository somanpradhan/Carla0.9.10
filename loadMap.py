import sys
import os
import glob

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass




import carla


def load_map():    
    """
    Load the map for the given Carla world.
    
    Args:
        world (carla.World): The Carla world instance.
    
    Returns:
        carla.Map: The loaded map.
    """    
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

# Load Town02
    # client.load_world('TownTestSpeed30')
    client.load_world('Town02')

# Optionally, get a handle to the new world
    world = client.get_world()
    print("Loaded map:", world.get_map().name)


if __name__ == "__main__":
    load_map()
