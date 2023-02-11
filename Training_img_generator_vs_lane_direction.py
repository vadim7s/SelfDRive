#!/usr/bin/env python



"""
This continues from Training_img_generator_with_autopilot.py
but
IDEA - use angle difference vs lane direction as a Y label

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


try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

WHITE = (255,255,255)
TIMER_LIMIT = 1000 #frames/ticks to limit recording loop to
EPISODES = 5 # not used currently
MIN_DISTANCE_FROM_JUNCTIONS = 20
MIN_SPEED_TO_CHANGE_YAW = 2 #yaw adjustments will not apply below this speed
SHOW_PYGAME = True #toggle to show display or not during image capture
MAX_STEER_ANGLE_FOR_ADJ = 0.01 #the max steering angle for a random adj to be done (to ensure adjustments are only done when the car is straight)
PREFERRED_CAR = 'charger_2020'
YAW_ADJ_DEGREES = 30
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

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

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
        vel = self.vehicle.get_velocity()
        speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        loc = self.vehicle.get_location() #returns something like Location(x=184.298294, y=-143.088516, z=0.000297)
        close_to_junction = False
        for x in self.junction_list:
            if loc.distance(x)<MIN_DISTANCE_FROM_JUNCTIONS:
                close_to_junction = True
        # now need to establish proximity to any junction
        if self.tics_processing>5 and close_to_junction == False: #this if ignores images of car being dropped down at spawn and when it is stops for lights
            # save camera image
            image.save_to_disk( '_out/%06d.png' % image.frame, carla.ColorConverter.Depth)
            # save steering input
            f = open('_out/%06d.str' % image.frame,'w')
            f.write(str(round(self.vehicle.get_control().steer,4)))
            f.close()
            
        if SHOW_PYGAME: #self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        # this was getting hight of the road vs hight of the sensor/car
        #sensor_height = self.sensor.get_location().z
        #road_height = self.transform.location.z
         
        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        

    
    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)
            throttle_value = 'Throttle: '+str(round(self.vehicle.get_control().throttle,4))
            steering_value = 'Steering: '+str(round(self.vehicle.get_control().steer,4))
            vel = self.vehicle.get_velocity()
            velo = 'Speed: '+str(round(3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2),4))
            font = pygame.font.SysFont(None, 24)
            throttle = font.render(throttle_value, True, WHITE)
            self.display_man.display.blit(throttle,(20, 20))
            steering = font.render(steering_value, True, WHITE)
            self.display_man.display.blit(steering,(20, 40))
            velocity = font.render(velo, True, WHITE)
            self.display_man.display.blit(velocity,(20, 60))
            
            
            
            

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
        client.load_world('Town05')
        time.sleep(15)

        world = client.get_world()
        
               
        traffic_manager = client.get_trafficmanager(8000)
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter(PREFERRED_CAR)[0]

        # get all intersections to avoid cutting images near them
        # as autopilot turning is not something we want
        junction_list = []
        waypoint_list = world.get_map().generate_waypoints(2.0)
        for x in waypoint_list:
            if x.get_junction() is not None:
                junction_list.append(x.transform.location)
                

        spawn_points = world.get_map().get_spawn_points()
        
        good_spawn_points = []

        for point in spawn_points:
            this_waypoint = world.get_map().get_waypoint(point.location,project_to_road=True, lane_type=(carla.LaneType.Driving))
            if this_waypoint.road_id in good_roads:
                good_spawn_points.append(point)
        
                    
        transform = random.choice(good_spawn_points)
        tick_counter=0
        # introducing an initial random yaw rotation at the start for the atopilot to correct (possibly redundant as other sdjustments are done later)
        transform.rotation.yaw = transform.rotation.yaw + random.randrange(-YAW_ADJ_DEGREES, YAW_ADJ_DEGREES, 1)
        vehicle = world.spawn_actor(bp, transform)

        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True)
        traffic_manager.set_desired_speed(vehicle,float(10))

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[1, 1], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        
        SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                    vehicle, {}, display_pos=[0, 0], junctions=junction_list)
        

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()
            tick_counter +=1
            # when the car is straight (steering is straight) and above certain speed introduce random yaw
            vel = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            steer = vehicle.get_control().steer
            if abs(steer) < MAX_STEER_ANGLE_FOR_ADJ and speed>MIN_SPEED_TO_CHANGE_YAW:
                trans = vehicle.get_transform()
                trans.rotation.yaw = trans.rotation.yaw + random.randrange(-YAW_ADJ_DEGREES, YAW_ADJ_DEGREES, 1)
                vehicle.set_transform(trans)
                
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break
            if call_exit or tick_counter>TIMER_LIMIT:
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
