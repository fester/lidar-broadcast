import yaml
from lidar import SubSocket
import numpy as np

with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)
    print(cfg)

print("Opening sub socket")
socket = SubSocket(cfg)

print("Receiving data")

for payload in socket:
    print("Received payload of {s} measurements taken at {t}".format(s=len(payload['scan']), t=payload['time']))
    scan = payload['scan']
    pts = np.array([(x['distance'], x['angle']) for x in scan])
    print(f"Median distance: {np.median(pts[:, 0]):.4f} (min {np.min(pts[:, 0]):.4f}, max: {np.max(pts[:, 0]):.4f}, +- {np.std(pts[:, 0]):.4f}")
    print(f"Median angle: {np.median(pts[:, 1]):.4f} (min {np.min(pts[:, 1]):.4f}, max: {np.max(pts[:, 1]):.4f}, +- {np.std(pts[:, 1]):.4f}")
