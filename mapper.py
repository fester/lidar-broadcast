import sys
import math
import yaml
import pygame
import numpy as np
import threading
import time
from scipy import misc, signal
from scipy.ndimage import gaussian_filter
import cv2

from lidar import SubSocket



g_Points = None
g_CutoffThreshold = 100
g_Zoom = 130
g_LastBlur = 5

def p2r(row):
    angle = np.deg2rad(row['angle'])
    x = row['distance']*math.cos(angle)
    y = -row['distance']*math.sin(angle)
    return np.complex(x, y)


def update_readings(socket):
    global g_Points
    for payload in socket:
         data = payload['scan']
         coords = map(p2r, filter(lambda r: r['strength'] > g_CutoffThreshold, data))
         g_Points = np.fromiter(coords, dtype=np.complex)

         
def render_lidar(surface, features, pts):
    global g_LastBlur

    if pts is None or len(pts) == 0:
        return

    kernel_width = 32
    kernel = gkern(kernel_width, 3.5)
    kernel_surface = pygame.surfarray.make_surface(kernel)
    
    for p in pts:
        x, y = p.real, p.imag
        sx -= kernel_width/2
        sy -= kernel_width/2
        surface.blit(kernel_surface, (sx, sy), pygame.BLEND_ADD)


    # arr = pygame.surfarray.pixels3d(surface)
    # w, h, _ = arr.shape
    # # if g_LastBlur == 0:
    # #     gaussian_filter(arr, sigma=0.8, output=arr)
    # #     g_LastBlur = 5

    # g_LastBlur -= 1

    # idx = np.stack([pts.real, pts.imag], axis=-1)
    # idx *= g_Zoom
    # idx += (w/2, h/2)
    # idx = np.rint(idx).astype(np.int)
    # idx = idx[(idx[:, 0] >= 0) & (idx[:, 0] < w) & (idx[:, 1] >= 0) & (idx[:, 1] < h)]

    # for x, y in idx:
    #     arr[x, y] = (255, 255 ,255)


    # gaussian_filter(arr, sigma=1, output=arr)
    
    # orb = cv2.ORB_create()
    # kp = orb.detect(arr,None)
    # kp, des = orb.compute(arr, kp)
    # img2 = cv2.drawKeypoints(arr, kp, outImage=None, color=(0, 255, 0), flags=0) #, color=(0,255,0), flags=0)
    # pygame.surfarray.blit_array(features, img2)

    
def process_keys(kb_state):
    global g_CutoffThreshold, g_Zoom
    changed = True
    
    if kb_state[pygame.K_DOWN]:
        g_CutoffThreshold = max(0, g_CutoffThreshold-5)
    elif kb_state[pygame.K_UP]:
        g_CutoffThreshold = min(255, g_CutoffThreshold+5)        
    elif kb_state[pygame.K_LEFT]:
        g_Zoom = max(1, g_Zoom-0.5)
    elif kb_state[pygame.K_RIGHT]:
        g_Zoom = g_Zoom+0.5
    else:
        changed = False

    return changed


def gkern(kernlen, nsig):
    """Returns a 2D Gaussian kernel array."""

    interval = (2*nsig+1.)/(kernlen)
    x = np.linspace(-nsig-interval/2., nsig+interval/2., kernlen+1)
    kern1d = np.diff(st.norm.cdf(x))
    kernel_raw = np.sqrt(np.outer(kern1d, kern1d))
    kernel = kernel_raw/kernel_raw.sum()
    return kernel


def main():
    with open("config.yaml", "rt") as f:
        cfg = yaml.load(f)


    lidar_socket = SubSocket(cfg)
    thread = threading.Thread(target=update_readings, args=(lidar_socket,))
    thread.start()

    pygame.init()

    size = width, height = 1024, 768

    screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
    screen.set_alpha(None)

    clock = pygame.time.Clock()

    lidar_screen = pygame.Surface(size)
    features = pygame.Surface(size)

    lidar_screen.fill((0,0,0))

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: break

        clear_required = process_keys(pygame.key.get_pressed())

        if clear_required:
            screen.fill((0,0,0))

        # lidar_screen.fill((0,0,0))
        features.fill((0,0,0))
    
        render_lidar(lidar_screen, features, g_Points)


        screen.blit(lidar_screen, (0,0))
        screen.blit(features, (0,0))
    
        pygame.display.flip()

        t = clock.tick()
        pygame.display.set_caption("MAPPER: FPS {fps:.2f}, CO: {co}, Z: {z:.4f}".format(fps=clock.get_fps(), co=g_CutoffThreshold, z=g_Zoom))
    


if __name__ == "__main__":
    main()
