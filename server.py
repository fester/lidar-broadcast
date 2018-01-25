import time
import yaml
from lidar import PubSocket
from lidar.scanner import TranslateReadings
import time

from sweeppy import Sweep

with open("./config.yaml", "rt") as f:
    cfg = yaml.load(f)

print("Opening pub socket")

pub = PubSocket(cfg)
initial_motor_speed = cfg.get('motor_speed', 1)
initial_sample_rate = cfg.get('sample_rate', 500)
print("Initial speed: {}, sampling rate: {}".format(initial_motor_speed, initial_sample_rate))

with Sweep(cfg['tty']) as sweep:
    sweep.set_motor_speed(initial_motor_speed)
    sweep.set_sample_rate(initial_sample_rate)
    while not sweep.get_motor_ready():
        print("Waiting for a lidar motor to become ready")
        time.sleep(1)

    print("Actual motor speed: {}, sampling rate: {}".format(sweep.get_motor_speed(), sweep.get_sample_rate()))

    sweep.start_scanning()
    print("All set, liftoff")

    pub.broadcast(TranslateReadings(sweep))
