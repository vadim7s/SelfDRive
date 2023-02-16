#!/usr/bin/env python



"""
This continues from Training_img_generator_with_autopilot.py
but because of poor label quality,
this is to take the car through good roads and
take photos in each while spinnining the car in each point


"""

import glob
import os
import sys
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np
import cv2


try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

WHITE = (255,255,255)
SHOW_PYGAME = True #toggle to show display or not during image capture
PREFERRED_CAR = 'model3'
YAW_ADJ_DEGREES = 45
# highway road ids in Town05
good_roads = [12, 34, 35, 36, 37, 38, 1201, 1236, 2034, 2035, 2343, 2344]

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self,angle):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render(angle)

        pygame.display.flip()
    


    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, junctions):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.junction_list = junctions

        self.time_processing = 0.0
        self.tics_processing = 0

        self.vehicle = attached
        self.transform = transform

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
                    
        if SHOW_PYGAME: #self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        # this was getting hight of the road vs hight of the sensor/car
        #sensor_height = self.sensor.get_location().z
        #road_height = self.transform.location.z
         
        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        

    
    def render(self,angle):
        if self.surface is not None:
            #  create a copy of the surface to save to disk
            view = pygame.surfarray.array3d(self.surface)
            #  convert from (width, height, channel) to (height, width, channel)
            view = view.transpose([1, 0, 2])
            #  convert from rgb to bgr
            img_gry = cv2.cvtColor(view, cv2.COLOR_RGB2GRAY) 
            time_grab = time.time_ns()
            cv2.imwrite('_out_ang/%06d_%s.png' % (time_grab, angle), img_gry)
            # save angle from straight
            
            font = pygame.font.SysFont(None, 24)
            angle_to_lane = font.render(str(angle), True, WHITE)
            self.display_man.display.blit(angle_to_lane,(20, 20))
            
            
            
            

    def destroy(self):
        self.sensor.destroy()

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        
        # this changes to specific map - can be commented out if server is changed already
        
        world = client.get_world()
        
               
        traffic_manager = client.get_trafficmanager(8000)
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter(PREFERRED_CAR)[0]

        town_map = world.get_map()
        # get all intersections to avoid cutting images near them
        # as autopilot turning is not something we want
        junction_list = []
        waypoint_list = town_map.generate_waypoints(2.0)
        for x in waypoint_list:
            if x.get_junction() is not None:
                junction_list.append(x.transform.location)
                

        spawn_points = town_map.get_spawn_points()
        
        good_spawn_points = []

        for point in spawn_points:
            this_waypoint = town_map.get_waypoint(point.location,project_to_road=True, lane_type=(carla.LaneType.Driving))
            if this_waypoint.road_id in good_roads:
                good_spawn_points.append(point)
        
                    
        transform = random.choice(good_spawn_points)
        tick_counter=0
        vehicle = world.spawn_actor(bp, transform)
        vehicle_list.append(vehicle)
        
        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[1, 1], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                    vehicle, {}, display_pos=[0, 0], junctions=junction_list)
        

        # set navigation points to loop through
        all_waypoint_pairs = town_map.get_topology()
        # subset of lane start/end's which belong to good roads
        good_lanes = []
        for w in all_waypoint_pairs:
            if w[0].road_id in good_roads:
                good_lanes.append(w)

        #Simulation loop
        call_exit = False
        # loop all lanes
        for lane in good_lanes:
            #loop within a lane
            for wp in lane[0].next_until_lane_end(20):
                transform = wp.transform
                vehicle.set_transform(transform)
                # do multiple shots of straight direction
                for i in range(5):
                    trans = wp.transform
                    angle_adj = random.randrange(-YAW_ADJ_DEGREES, YAW_ADJ_DEGREES, 1)
                    trans.rotation.yaw = transform.rotation.yaw +angle_adj 
                    vehicle.set_transform(trans)
                    # Carla Tick
                    if args.sync:
                        world.tick()
                    else:
                        world.wait_for_tick()

                    # Render received data
                    display_manager.render(angle_adj)
                    tick_counter +=1
                    # move the car to location
                    vehicle.set_transform(transform)
    
                        
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            call_exit = True
                        elif event.type == pygame.KEYDOWN:
                            if event.key == K_ESCAPE or event.key == K_q:
                                call_exit = True
                                break
                    if call_exit:
                        break

    finally:
        if display_manager:
            display_manager.destroy()
      
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        



def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
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
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='640x360',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)
        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
