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


class ScanseLidar(Laser):
    ENGINE_ROTATION_HZ = 6
    SAMPLING_RATE_HZ = 1000
    SAMPLES_COUNT = int(1.3*SAMPLING_RATE_HZ/ENGINE_ROTATION_HZ)
    HOLE_WIDTH = 40
    
    def __init__(self):
        # A 360 degree lidar scanning points between NO_DETECTION_THESHOLD and RANGE mm
        Laser.__init__(self, self.SAMPLES_COUNT, self.ENGINE_ROTATION_HZ, 360, self.HOLE_WIDTH, offset_mm=25)


def convert_point(angle, distance, strength):
    angle_r = angle*math.pi/180.0
    x = distance*math.cos(angle_r)
    y = -distance*math.sin(angle_r)
    return (x, y)


def read_scans(data_source):
    deg_per_scan = 360/ScanseLidar.SAMPLES_COUNT

    for scan in data_source:
        payload = scan['scan']
        points = ((p['angle']%360, p['distance'], p['strength']) for p in payload)
        
        points = np.array(list(points))

        #threshold = np.percentile(points[:, 2], 30)
        threshold = 0
        points = points[points[:, 2] > threshold]
        points[:, 1] *= 1000 # Scale distance from meters to mm
        
        distance_bins = [0] * ScanseLidar.SAMPLES_COUNT
        
        for p in points:
            bin_no = int(p[0]/deg_per_scan)
            distance_bins[bin_no] = p[1] # Copy distance value at an angle to a resulting bin

        yield distance_bins


def render_map(screen, slam_map):
    screen.fill((0,0,0))
    
    map_ps = np.array(slam_map).reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

    render_plane = imresize(map_ps, (SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS))

    render_plane = np.dstack([render_plane, render_plane, render_plane])
    pygame.surfarray.blit_array(screen, render_plane)
    

def render_bot_reference(surface, x, y, theta):

    if x < 14000:
        import pdb; pdb.set_trace()
    
    scale = (MAP_SIZE_PIXELS/(MAP_SIZE_METERS*1000))
    arrow_length = 20 # arrow length in pixels
    
    x = x*scale
    y = y*scale

    sx = x*(SCREEN_SIZE_PIXELS/MAP_SIZE_PIXELS)
    sy = y*(SCREEN_SIZE_PIXELS/MAP_SIZE_PIXELS)
    
    ex = sx+arrow_length*math.cos(theta)
    ey = sy+arrow_length*math.sin(theta)

    pygame.draw.circle(surface, (0, 255, 0), (int(sx), int(sy)), int(arrow_length/2), 3)
    pygame.draw.line(surface, (255, 0, 0), (sx, sy), (ex, ey))

    
def do_the_slam_thing(slam, mapbytes, data_source):
    screen = pygame.display.set_mode((SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS), pygame.DOUBLEBUF)

    reads_count = 0
    x, y = 0, 0

    for scan in data_source:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()

        slam.update(scan, (0, 0, 0))
        x, y, theta = slam.getpos()
        slam.getmap(mapbytes)

        render_map(screen, mapbytes)
        print(x,y,theta)
        render_bot_reference(screen, x, y, theta)
        
        pygame.display.flip()

        reads_count += 1
        if reads_count % SCANS_THRESHOLD == 0:
            print("... {} scans, est'd position: {}, {}, angle {}".format(reads_count, x, y, theta))
            with open("map_updates/map_{}".format(reads_count), "wb") as f:
                f.write(mapbytes)
            
        
def main():
    print("Initializing lidar server")
    pygame.init()
        
    with open("./config.yaml", "rt") as f:
        cfg = yaml.load(f)

    socket = SubSocket(cfg)
    lidar = ScanseLidar()

    print("Initializing SLAM")

    mapbytes = bytearray(MAP_SIZE_PIXELS**2)
    slam = RMHC_SLAM(lidar, MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    print("We're set for liftoff. reading")

    do_the_slam_thing(slam, mapbytes, read_scans(socket))
    
            
                       
                       
    
if __name__ == '__main__':
    main()
                    
