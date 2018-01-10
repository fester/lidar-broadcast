import zmq
import pickle

class PubSocket:
    def __init__(self, config):
        self.__config = config
        self.__setup_socket()

    def __setup_socket(self):
        ctx = zmq.Context()
        s = ctx.socket(zmq.PUB)
        host = self.__config.get('host', '*')
        port = self.__config.get('port', 5566)
        
        bind_addr = f"tcp://{host}:{port}"
        s.bind(bind_addr)

        self.__context = ctx
        self.__socket = s

    def __to_wire(self, data):
        return pickle.dumps(data)
        
    def __publish(self, payload):
        data = self.__to_wire(payload)
        self.__socket.send(data)
        
    def broadcast(self, data_source):
        for chunk in data_source:
            self.__publish(chunk)
