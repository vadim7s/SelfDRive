#from CARLA camera tutorial on YouTube
# this approach make the camera image available with for a simple loop

import carla
import math
import time
import cv2
import numpy as np
import random

PREFERRED_SPEED = 10
#mount point of camera on the car
CAMERA_POS_Z = 1.6
CAMERA_POS_X = 0.9


client = carla.Client('localhost', 2000)
client.set_timeout(10)
client.load_world('Town05') 


world = client.get_world()

traffic_manager = client.get_trafficmanager(8000)
settings = world.get_settings()
traffic_manager.set_synchronous_mode(True)
# option preferred speed
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter('*model3*')

town_map = world.get_map()

'''
#old distance based approach 
preferred_start_point = carla.Location(x=-247.802231, y=-102.741714, z=10.000187)
start_points = world.get_map().get_spawn_points()
start_point = start_points[0]
distance = carla.Location.distance(preferred_start_point,start_point.location)
for sp in start_points:
    current_distance = carla.Location.distance(sp.location,preferred_start_point)
    if current_distance < distance:
        distance = current_distance
        start_point = sp
'''
good_roads = [37]
spawn_points = world.get_map().get_spawn_points()
good_spawn_points = []
for point in spawn_points:
    this_waypoint = world.get_map().get_waypoint(point.location,project_to_road=True, lane_type=(carla.LaneType.Driving))
    if this_waypoint.road_id in good_roads:
        good_spawn_points.append(point)

start_point = random.choice(good_spawn_points)

vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)
traffic_manager.set_desired_speed(vehicle,float(PREFERRED_SPEED))


#setting RGB Camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
camera_bp.set_attribute('image_size_y', '360')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
camera.listen(lambda image: camera_callback(image,camera_data))


cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera',camera_data['image'])
cv2.waitKey(1)

#main loop 
quit = False
vehicle.set_autopilot(True)
while True:
    # Carla Tick
    world.tick()

    if cv2.waitKey(1) == ord('q'):
        quit = True
        break
    cv2.imshow('RGB Camera',camera_data['image'])
    
            
#clean up
cv2.destroyAllWindows()
camera.stop()
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()