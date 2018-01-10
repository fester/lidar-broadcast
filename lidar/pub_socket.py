import zmq

class PubSocket:
    def __init__(self, config):
        self.__config = config
        self.__setup_socket()

    def __setup_socket(self):
        ctx = zmq.Context()
        s = ctx.socket(zmq.PUB)
        host = self.__config.get('host', '*')
        port = self.__config.get('port', 5566)
        
        bind_addr = "tcp://{host}:{port}".format(host=host, port=port)
        s.bind(bind_addr)

        self.__context = ctx
        self.__socket = s

    def __publish(self, payload):
        # data = self.__to_wire(payload)
        self.__socket.send_json(payload)
        
    def broadcast(self, data_source):
        for chunk in data_source:
            self.__publish(chunk)
