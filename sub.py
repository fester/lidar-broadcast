import yaml
from lidar import SubSocket

with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)
    print(cfg)

print("Opening sub socket")
socket = SubSocket(cfg)

print("Receiving data")
for payload in socket:
    print("Received payload of {s} measurements taken at {t}".format(s=len(payload['scan']), t=payload['time']))
