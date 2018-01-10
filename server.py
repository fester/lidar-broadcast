import yaml
from lidar import PubSocket
from lidar.scanner import TranslateReadings

from sweeppy import Sweep


with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)

print("Opening pub socket")


pub = PubSocket(cfg)

with Sweep(cfg['tty']) as sweep:
    sweep.start_scanning()
    pub.broadcast(TranslateReadings(sweep))
