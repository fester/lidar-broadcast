from threading import Thread
import time
import sys
import yaml
import math
import pygame
import numpy as np
from scipy.misc import imresize, imsave
from scipy import optimize as opt
import cv2

from breezyslam.algorithms import RMHC_SLAM, distanceScanToMap
from breezyslam.sensors import Laser
import pybreezyslam

from lidar import SubSocket
from lidar.navigator import OccupancyGrid
from lidar.controller import BotController


MAP_SIZE_PIXELS = 4000
MAP_SIZE_METERS = 20
SCREEN_SIZE_PIXELS = 900

g_Slam = {'core': None, 'scan': [], 'map': None, 'pos': (0, 0, 0), 'path': {'goal': None, 'route': None}}
g_Quitting = False
g_Controller = None

class ScanseLidar(Laser):
    ENGINE_ROTATION_HZ = 6
    SAMPLING_RATE_HZ = 1000
    SAMPLES_COUNT = int(1*SAMPLING_RATE_HZ/ENGINE_ROTATION_HZ)
    HOLE_WIDTH = 500

    def __init__(self):
        # A 360 degree lidar scanning points between NO_DETECTION_THESHOLD and RANGE mm
        Laser.__init__(self, self.SAMPLES_COUNT, self.ENGINE_ROTATION_HZ, 360, self.HOLE_WIDTH, offset_mm=25)


class A2Lidar(Laser):
    ENGINE_ROTATION_HZ = 10
    SAMPLING_RATE_HZ = 4000
    SAMPLES_COUNT = int(SAMPLING_RATE_HZ/ENGINE_ROTATION_HZ)
    HOLE_WIDTH = 200
    
    def __init__(self):
        Laser.__init__(self, self.SAMPLES_COUNT, self.ENGINE_ROTATION_HZ, 360, A2Lidar.HOLE_WIDTH, offset_mm=-25)
        

def read_scans(data_source):
    global g_Slam

    deg_per_scan = 360/A2Lidar.SAMPLES_COUNT
    for scan in data_source:

        if g_Quitting:
            break

        payload = scan['scan']
        distance_bins = [0] * A2Lidar.SAMPLES_COUNT

        # Copy distance value at an angle to a resulting bin
        for p in payload:
            if p['strength'] < 5:
                continue
            a = p['angle']
            d = p['distance']

            bin_no = int((a/360)*A2Lidar.SAMPLES_COUNT)
            distance_bins[bin_no] = d

        g_Slam['scan'] = distance_bins


def render_map(screen, slam_map):
    screen.fill((0, 0, 0))

    map_ps = np.array(slam_map).reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    render_plane = imresize(map_ps, (SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS))
    render_plane = np.dstack([render_plane, render_plane, render_plane])
    pygame.surfarray.blit_array(screen, render_plane)


def render_bot_reference(surface):
    x, y, theta = g_Slam['pos']

    map_mm = MAP_SIZE_METERS*1000

    arrow_length = 20  # arrow length in pixels

    sx = (x/map_mm)*SCREEN_SIZE_PIXELS
    sy = (y/map_mm)*SCREEN_SIZE_PIXELS

    ex = sx+arrow_length*math.cos(theta)
    ey = sy+arrow_length*math.sin(theta)

    pygame.draw.circle(surface, (0, 255, 0), (int(sy), int(sx)),
                       int(arrow_length/2), 3)
    pygame.draw.line(surface, (255, 0, 0), (sy, sx), (ey, ex))


def do_the_slam_thing():
    global g_Slam

    slam = g_Slam['core']
    loops_count = 0

    while not g_Quitting:
        scan = g_Slam['scan']

        if not scan:
            continue

        slam.update(scan, (0, 0, 0))
        g_Slam['pos'] = slam.getpos()
        slam.getmap(g_Slam['map'])

        loops_count += 1
        if loops_count % 100 == 0:
            with open(f"map_updates/map_{MAP_SIZE_METERS}_{MAP_SIZE_PIXELS}_{loops_count:04}.bin", "wb") as f:
                f.write(g_Slam['map'])


# OPTIMIZATION AND RELOCALIZATION - SOME WHEELS WERE REINVENTED
                
class UnableToLocalise(Exception):
    pass

WORLD_TO_MAP_SCALE=(MAP_SIZE_METERS*1000)/MAP_SIZE_PIXELS


def world_to_map(*args):
    if len(args) == 2:
        x, y = args
    elif len(args) == 1:
        x, y = args[0]
    else:
         raise TypeError("invalid arguments")  
        
    return int(x/WORLD_TO_MAP_SCALE), int(y/WORLD_TO_MAP_SCALE)


def map_to_world(*args):
    if len(args) == 2:
        x, y = args
    elif len(args) == 1:
        x, y = args[0]
    else:
         raise TypeError("invalid arguments")      
    
    return x*WORLD_TO_MAP_SCALE, y*WORLD_TO_MAP_SCALE


def polar_to_cartesian(scan):
    samples_no = len(scan)
    deg_per_sample = 360/samples_no
    
    def p2c(a, d):
        a = np.deg2rad(a)
        return d*math.cos(a), d*math.sin(a)
    
    ep = ((a*deg_per_sample, d) for a, d in enumerate(scan) if d != 0)
    ep = [p2c(a, d) for a, d in ep]

    return np.array(ep)
        

def transform_scan(scan, X):
    if not isinstance(scan, np.ndarray):
        scan = np.array(scan)
    
    ones = np.ones((len(scan), 1))
    scan = np.hstack((scan, ones))  # Homogenise coordinates
    
    
    x, y, t = X
    t = np.deg2rad(t)
    ct = math.cos(t)
    st = math.sin(t)

    m = np.matrix([
        [ct, -st, 0],
        [st,  ct, 0],
        [x,    y, 1]
    ])

    return np.asarray((np.asmatrix(scan)*m)[..., 0:2])


def scan_to_map_coords(scan):
    return np.apply_along_axis(world_to_map, 1, scan)


def cost_fn(distmap, pts):
    w, h = distmap.shape
    vs = np.fromiter((distmap[y, x]**2 for x, y in pts
          if (0 <= x < w) and (0 <= y < w)), dtype=np.float)
    return  np.average(vs) #  sum(vs)/len(vs)


def target_fn(X, scan, distance_map):
    scan = transform_scan(scan, X)
    map_pixels = scan_to_map_coords(scan)
    return cost_fn(distance_map, map_pixels)


def perform_initial_localisation(slam, scan):
    floor_plan = np.fromiter(iter(g_Slam['map']), dtype=np.uint8).reshape(
        (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

    bin_plan = np.where(floor_plan < 64, 0, 255).astype(np.uint8)
    camfer_plan = cv2.distanceTransform(bin_plan, cv2.DIST_L2, 5)    
    
    border = 500
    bounds = ((border, MAP_SIZE_METERS*1000-border),
              (border, MAP_SIZE_METERS*1000-border),
              (-180, 180))

    print("Begin optimisation")
    opts = []
    for i in range(10):
        c_scan = polar_to_cartesian(g_Slam['scan'])

        opt_res = opt.differential_evolution(target_fn, bounds, args=(c_scan, camfer_plan),
                                             polish=True)
        g_Controller.rotate(360/10)
        opts.append(opt_res.x)
    print("end optimisation, so far we've {}".format(opts))
    

    if not opt_res.success:
        raise UnableToLocalise("Can't come up with a proper solution")

    pos = list(opt_res.x)

    slam.setpos(pos)
    print(f"POS {slam.getpos()}")
    return pos


def start_slam():
    global g_Slam

    lidar = A2Lidar()

    need_primary_localisation = True

    if not g_Slam['map']:
        g_Slam['map'] = bytearray([127]*MAP_SIZE_PIXELS**2)
        need_primary_localisation = False

    g_Slam['core'] = RMHC_SLAM(lidar, MAP_SIZE_PIXELS, MAP_SIZE_METERS, map_bytes=g_Slam['map'],
                               max_search_iter=10000, hole_width_mm=500, sigma_xy_mm=200)

    while not g_Slam['scan']:  # Wait till an actual scan arrives
        time.sleep(0.01)

    # Localise bot on an existing map if needed
    if need_primary_localisation:
        plp = perform_initial_localisation(g_Slam['core'], g_Slam['scan'])


    t = Thread(target=do_the_slam_thing)
    t.start()


def start_scanning(cfg):
    scan_stream = SubSocket(cfg)
    t = Thread(target=read_scans, args=(scan_stream,))
    t.start()


def render_loop():
    global g_Quitting
    screen = pygame.display.set_mode((SCREEN_SIZE_PIXELS, SCREEN_SIZE_PIXELS),
                                     pygame.DOUBLEBUF)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                g_Quitting = True
                return

        mapbytes = g_Slam['map']

        render_map(screen, mapbytes)
        render_bot_reference(screen)
        pygame.display.flip()

        
def start_navigation():
    world_map = g_Slam['map']
    p = g_Slam['pos']
    GRID_DENSITY = 16
    grid = OccupancyGrid(world_map, MAP_SIZE_METERS, 16)

    k1 = MAP_SIZE_PIXELS/(MAP_SIZE_METERS*1000)
    px = (p[0] * k1)/GRID_DENSITY
    py = (p[1] * k1)/GRID_DENSITY

    path = grid.get_path((93, 93), (0, 0))

    map_space_path = map(lambda c: grid.to_map_coords(c), path)

    
def main():
    global g_Controller
    print("Initializing lidar server")

    with open("./config.yaml", "rt") as f:
        cfg = yaml.load(f)

    print("Initializing SLAM")

    if len(sys.argv) == 2:
        with open(sys.argv[1], "rb") as f:
            g_Slam['map'] = map_ = bytearray(f.read())
            assert len(map_) == MAP_SIZE_PIXELS**2

    g_Controller = BotController("http://belka:5000")
            
    pygame.init()

    start_scanning(cfg)
    start_slam()

    time.sleep(0.5)

    # g_Controller.move(0.5)
    # g_Controller.rotate(90)

    # g_Controller.move(0.5)
    # g_Controller.rotate(90)

    # g_Controller.move(0.5)
    # g_Controller.rotate(90)

    # g_Controller.move(0.5)
    # start_navigation()
    render_loop()


if __name__ == '__main__':
    main()
