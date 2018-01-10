import time

class TranslateReadings:
    def __init__(self, sweeper):
        self.__sweep_generator = sweeper.get_scans()
        
    def __transform_sample(self, sample):
        return {'angle': sample.angle/1000.0,
                'distance': sample.distance/100.0,
                'strength': sample.signal_strength}
        
    def __scan_data(self, scan):
        return list(map(self.__transform_sample, scan.samples))

    def __iter__(self):
        return self

    def __next__(self):
        scan = next(self.__sweep_generator)
        timestamp = time.time()
        return {"time": timestamp,
                "scan": self.__scan_data(scan)}
