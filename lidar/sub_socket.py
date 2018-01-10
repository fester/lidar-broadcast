import zmq

class SubSocket:
    def __init__(self, config):
        self.__config = config
        self.__setup_socket()

    def __setup_socket(self):
        ctx = zmq.Context()
        s = ctx.socket(zmq.SUB)

        host = self.__config.get('host', '127.0.0.1')
        port = self.__config.get('port', 5566)
        connect_addr = "tcp://{host}:{port}".format(host=host, port=port)
        s.connect(connect_addr)
        s.setsockopt(zmq.SUBSCRIBE, b'')
        self.__socket = s
        self.__context = ctx

    def __iter__(self):
        return self

    def __receive(self):
        data = self.__socket.recv_json()
        return data

    def next(self):
        return self.__next__()
    
    def __next__(self):
        payload = self.__receive()
        return payload
    
