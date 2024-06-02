import abc
import glob
import os
import sys
from types import LambdaType
from collections import deque
from collections import namedtuple
import carla
import time
import numpy as np
 
try:
#input path of carla
    sys.path.append(glob.glob('/home/xjc/carla/CARLA_0.9.13/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
 
client = carla.Client('localhost',2000)
client.set_timeout(10.0)
# Load the 'Town02' map
client.load_world('Town02')
print("Loaded 'Town02' map.")

world = client.get_world()
blueprint_library = world.get_blueprint_library()
model_3 = blueprint_library.filter('model3')[0]

pedestrian_bp = blueprint_library.filter("walker.pedestrian.*")[0]
spawn_point = carla.Transform(carla.Location(x=180, y=109, z=0.24))
pedestrian = world.spawn_actor(pedestrian_bp, spawn_point)  # Store as an instance attribute
print(f"Spawned pedestrian at x={spawn_point.location.x}, y={spawn_point.location.y}, z={spawn_point.location.z}")

actor_list = []
initial_spawn_point = carla.Transform(carla.Location(x=160, y=109, z=0.24), carla.Rotation(yaw=0)) 
vehicle = world.spawn_actor(model_3, initial_spawn_point)

# Now set the velocity
velocity = 11  

actor_list.append(vehicle)
def process_radar(mesure):
    global radar_data
    if hasattr(mesure, 'raw_data'):
        radar_data = mesure
    else:
        print("Received incomplete radar data.")

spectator = world.get_spectator()
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-10, z=10),carla.Rotation(pitch=-15)))

radar = blueprint_library.find('sensor.other.radar')
radar.set_attribute('horizontal_fov', '10')
radar.set_attribute('vertical_fov', '30')
radar.set_attribute('range', '100')
radar.set_attribute('points_per_second', '500000')
radar.set_attribute('sensor_tick', '0.1')
transform = carla.Transform(carla.Location(x=0 ,z=1.5 ))
radar_sensor = world.spawn_actor(radar,transform,attach_to=vehicle)
actor_list.append(radar_sensor)
radar_sensor.listen(lambda mesure:process_radar(mesure))

while 'radar_data' not in globals():
    time.sleep(0.1)  # wait for 1000 milliseconds before checking again

try:
    points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(radar_data), 4))
except AttributeError as e:
    print(f"Error while processing radar data: {e}")

# print(points)

np.savetxt(r'Radar_raw_data.txt', points, fmt='%s', delimiter=',') # save the radar raw data into Radar_raw_data.txt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib

matplotlib.use('Qt5Agg')  # Set the backend before importing pyplot

l=np.cos(points[:,2])*points[:,3]
z=np.sin(points[:,2])*points[:,3]
y=np.cos(points[:,1])*l
x=np.sin(points[:,1])*l
v=points[:, 0]
n_raw = len(v)
print(f"number of raw points: {n_raw}")

# plot the filtered radar points 
plt.figure("3D Scatter", facecolor="lightgray",figsize=(20,20),dpi=80)
ax3d = plt.gca(projection="3d")
ax3d.scatter(x, y, z, s=10, cmap="jet", marker="o") # raw plot
ax3d.view_init(elev=0, azim=-70)
ax3d.set_yticks(np.arange(0, 100, 10))
plt.show()

