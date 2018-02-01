import sys
import math
import yaml
import pygame
import numpy as np
import threading
import time
import scipy.stats as st
from scipy import misc, signal
from scipy.ndimage import gaussian_filter
import cv2

from lidar import SubSocket



g_Points = None
g_CutoffThreshold = 100
g_Zoom = 1000
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


def np_add(b1, b2, pos_v, pos_h):
    pos_v = int(pos_v)
    pos_h = int(pos_h)
    v_range1 = slice(max(0, pos_v), max(min(pos_v + b2.shape[0], b1.shape[0]), 0))
    h_range1 = slice(max(0, pos_h), max(min(pos_h + b2.shape[1], b1.shape[1]), 0))

    v_range2 = slice(max(0, -pos_v), min(-pos_v + b1.shape[0], b2.shape[0]))
    h_range2 = slice(max(0, -pos_h), min(-pos_h + b1.shape[1], b2.shape[1]))

    ranges = [v_range1, v_range2, h_range1, h_range2]
    bounds = (r.stop-r.start for r in ranges)
    negative_bounds = (d for d in bounds if d <= 0)

    if any(negative_bounds):
        return b1
    b1[v_range1, h_range1] += b2[v_range2, h_range2]
    return b1

         
def render_lidar(surface, features, pts):
    global g_LastBlur

    if pts is None or len(pts) == 0:
        return

#    import pdb; pdb.set_trace()
    
    
    w, h = surface.get_width(), surface.get_height()
    accumulator = np.zeros((w, h))

    pts = np.stack([pts.real, pts.imag], axis=-1)
    pts *= g_Zoom
    pts += (w/2, h/2)
    
    for p in pts:
        sx, sy = p
        sx -= kernel_width/2
        sy -= kernel_width/2
        #surface.blit(kernel_surface, (sx, sy), None, pygame.BLEND_ADD)
        # accumulator = np_add(accumulator, kernel, sx, sy)
        if sx < 0 or sx >= w or sy < 0 or sy >= h:
            continue
        accumulator[int(sx), int(sy)] = 255
        #surface.blit(kernel_surface, (sx, sy), None, pygame.BLEND_ADD)

    pixels = np.dstack([accumulator, accumulator, accumulator])
    pygame.surfarray.blit_array(surface, pixels)
    # accumulated_colors = np.rint(np.clip(accumulator, 0.0, 1.0)*255).astype(np.uint8)
    # accumulated_colors = np.stack([accumulated_colors, accumulated_colors, accumulated_colors], axis=-1)
    
    # pygame.surfarray.blit_array(surface, accumulated_colors)
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
        g_Zoom = max(1, g_Zoom-10)
    elif kb_state[pygame.K_RIGHT]:
        g_Zoom = g_Zoom+10
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
    return kernel/np.max(kernel)


kernel_width = 32
kernel = gkern(kernel_width, 2)
kernel = np.stack([kernel, kernel, kernel], axis=-1).astype(np.uint8)
kernel_surface = pygame.surfarray.make_surface(kernel)

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
        screen.fill((0,0,0))
    
        render_lidar(screen, features, g_Points)

        # screen.blit(lidar_screen, (0,0))
        # screen.blit(features, (0,0))
    
        pygame.display.flip()

        t = clock.tick()
        pygame.display.set_caption("MAPPER: FPS {fps:.2f}, CO: {co}, Z: {z:.4f}".format(fps=clock.get_fps(), co=g_CutoffThreshold, z=g_Zoom))
    


if __name__ == "__main__":
    main()
