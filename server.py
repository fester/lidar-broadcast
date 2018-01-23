import yaml
from lidar import PubSocket
from lidar.scanner import TranslateReadings
import time

from sweeppy import Sweep

with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)

print("Opening pub socket")

pub = PubSocket(cfg)

with Sweep(cfg['tty']) as sweep:
    print("Setting the motor settings up")
    sweep.set_motor_speed(cfg.get('motor_speed', 2))
    sweep.set_sample_rate(cfg.get('scaning_frequency', 500))
    while not sweep.get_motor_ready():
        print("Waiting for a sweep motor to become ready...")
        time.sleep(2)
    sweep.start_scanning()
    print("All set, liftoff")

    pub.broadcast(TranslateReadings(sweep))
