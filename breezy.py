from threading import Thread
import time
import sys
import yaml
import math
import pygame
import numpy as np
from scipy.misc import imresize


from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

from lidar import SubSocket

SCANS_THRESHOLD = 10
MAP_SIZE_PIXELS = 3000
MAP_SIZE_METERS = 30
SCREEN_SIZE_PIXELS = 900

g_Slam = {'core': None, 'scan': [], 'map': None, 'pos': (0, 0, 0)}
g_Quitting = False

class ScanseLidar(Laser):
    ENGINE_ROTATION_HZ = 6
    SAMPLING_RATE_HZ = 1000
    SAMPLES_COUNT = int(1*SAMPLING_RATE_HZ/ENGINE_ROTATION_HZ)
    HOLE_WIDTH = 40
    
    def __init__(self):
        # A 360 degree lidar scanning points between NO_DETECTION_THESHOLD and RANGE mm
        Laser.__init__(self, self.SAMPLES_COUNT, self.ENGINE_ROTATION_HZ, 360, self.HOLE_WIDTH, offset_mm=25)


class A2Lidar(Laser):
    ENGINE_ROTATION_HZ = 10
    SAMPLING_RATE_HZ = 4000
    SAMPLES_COUNT = int(SAMPLING_RATE_HZ/ENGINE_ROTATION_HZ)
    HOLE_WIDTH = 20

    def __init__(self):
        Laser.__init__(self, self.SAMPLES_COUNT, self.SAMPLING_RATE_HZ, 360, self.HOLE_WIDTH, offset_mm=25)
        

def convert_point(angle, distance, strength):
    angle_r = angle*math.pi/180.0
    x = distance*math.cos(angle_r)
    y = -distance*math.sin(angle_r)
    return (x, y)


def read_scans(data_source):
    global g_Slam
    
    deg_per_scan = 360/A2Lidar.SAMPLES_COUNT
    
    for scan in data_source:
        if g_Quitting:
            break
        
        payload = scan['scan']
        points = (((360-p['angle'])%360, p['distance'], p['strength']) for p in payload)
        
        points = np.array(list(points))
        
        distance_bins = [0] * A2Lidar.SAMPLES_COUNT
        
        for p in points:

            bin_no = int(p[0]/deg_per_scan)
            distance_bins[bin_no] = p[1] # Copy distance value at an angle to a resulting bin

        g_Slam['scan'] = distance_bins


def render_map(screen, slam_map):
    screen.fill((0,0,0))
    
    map_ps = np.array(slam_map).reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    
    render_plane = imresize(map_ps, (SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS))

    render_plane = np.dstack([render_plane, render_plane, render_plane])

    pygame.surfarray.blit_array(screen, render_plane)
    

def render_bot_reference(surface):
    x, y, theta = g_Slam['pos']

    map_mm = MAP_SIZE_METERS*1000
    
    scale = (MAP_SIZE_PIXELS/map_mm)
    arrow_length = 20 # arrow length in pixels

    x = x-(map_mm/2)
    y = y-(map_mm/2)

    x = x*scale
    y = y*scale

    sx = SCREEN_SIZE_PIXELS/2 + x*(SCREEN_SIZE_PIXELS/MAP_SIZE_PIXELS)
    sy = SCREEN_SIZE_PIXELS/2 - y*(SCREEN_SIZE_PIXELS/MAP_SIZE_PIXELS)
    
    ex = sx+arrow_length*math.cos(theta)
    ey = sy+arrow_length*math.sin(theta)

    pygame.draw.circle(surface, (0, 255, 0), (int(sy), int(sx)), int(arrow_length/2), 3)
    pygame.draw.line(surface, (255, 0, 0), (sy, sx), (ey, ex))


def do_the_slam_thing():
    global g_Slam

    reads_count = 0
    x, y = 0, 0

    slam = g_Slam['core']
    
    while not g_Quitting:
        if not g_Slam['scan']:
            continue
        
        slam.update(g_Slam['scan'], (0, 0, 0))
        g_Slam['pos'] = slam.getpos()
        
        slam.getmap(g_Slam['map'])
        time.sleep(0.1)
            

def start_slam():
    global g_Slam

    lidar = A2Lidar()
    g_Slam['core'] = RMHC_SLAM(lidar, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
    t = Thread(target=do_the_slam_thing)
    t.start()


def start_scanning(cfg):
    scan_stream = SubSocket(cfg)
    t = Thread(target=read_scans, args=(scan_stream,))
    t.start()
    
    
def render_loop():
    global g_Quitting
    screen = pygame.display.set_mode((SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS), pygame.DOUBLEBUF)

    reads_count = 0
    x, y = 0, 0

    while True:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                g_Quitting = True
                return

        mapbytes = g_Slam['map']

        render_map(screen, mapbytes)
        render_bot_reference(screen)
        
        pygame.display.flip()
    
                
def main():
    print("Initializing lidar server")
        
    with open("./config.yaml", "rt") as f:
        cfg = yaml.load(f)

    print("Initializing SLAM")

    g_Slam['map'] = bytearray(MAP_SIZE_PIXELS**2)

    pygame.init()


    start_scanning(cfg)
    start_slam()
    
    render_loop()
                       
                       
    
if __name__ == '__main__':
    main()
                    
