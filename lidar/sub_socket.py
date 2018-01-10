import zmq
import pickle

class SubSocket:
    def __init__(self, config):
        self.__config = config
        self.__setup_socket()

    def __setup_socket(self):
        ctx = zmq.Context()
        s = ctx.socket(zmq.SUB)

        host = self.__config.get('host', '127.0.0.1')
        host = self.__config.get('host', '127.0.0.1')
        port = self.__config.get('port', 5566)
        connect_addr = f"tcp://{host}:{port}"
        s.connect(connect_addr)
        s.setsockopt(zmq.SUBSCRIBE, b'')
        self.__socket = s
        self.__context = ctx

    def __from_wire(self, payload):
        return pickle.loads(payload)
        
    def __iter__(self):
        return self

    def __receive(self):
        data = self.__socket.recv()
        return self.__from_wire(data)
    
    def __next__(self):
        payload = self.__receive()
        return payload
    
