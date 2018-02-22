from threading import Thread
from queue import Queue, Empty
import time

import requests

K = 3.304
O = 0.04311

SECONDS_PER_FULL_ROTATION = 2.760

def distance_to_time(meters):
    return meters*K+O


def rotation_to_time(degrees):
    return degrees*SECONDS_PER_FULL_ROTATION/360

class BotController:
    MOVE = 1
    ROTATE = 2
    
    def __init__(self, api_url):
        self._queue = Queue()
        self._api_url = api_url
        self._keep_processing = True
        self._executor = Thread(target=self._process_command_queue)
        self._executor.start()
        
    def rotate(self, angle):
        self._queue.put((BotController.ROTATE, angle))
        
    def move(self, distance):
        self._queue.put((BotController.MOVE, distance))

    def shutdown(self):
        self._keep_processing = False
        
    def _process_command_queue(self):
        while self._keep_processing:
            try:
                moved = False
                cmd_kind, arg = self._queue.get(timeout=2)

                if cmd_kind == BotController.MOVE:
                    moved = True
                    self._send_command("forward")
                    time.sleep(distance_to_time(arg))

                elif cmd_kind == BotController.ROTATE:
                    moved = True

                    if arg > 0:
                        self._send_command("turnleft")
                        time.sleep(rotation_to_time(arg))
                    else:
                        self._send_command("turnright")
                        time.sleep(rotation_to_time(-arg))
            except Empty:
                pass
            finally:
                if moved:
                    self._send_command("stop")

    

    def _send_command(self, kind):
        r = requests.post(self._api_url+"/"+kind+"/")
        assert r.ok
        
