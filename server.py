import yaml
import random
import time
from lidar import PubSocket, TranslateReadings
import logging


def data_source():
    while True:
        pts = random.randrange(5, 100)
        angles = (random.randint(0, 360000) for _ in range(pts))
        distances = (random.randint(10, 40*100) for _ in range(pts))
        data = zip(angles, distances)
        yield list(data)
        time.sleep(1)


with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)

print("Opening pub socket")


pub = PubSocket(cfg)


with Sweep('/dev/ttyUSB0') as sweep:
    sweep.start_scanning()
    pub.boradcast(TranslateReadings(sweep))
