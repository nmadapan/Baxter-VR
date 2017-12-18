import socket, time, os, random

TCP_IP = '128.46.125.59'
TCP_PORT = 5000
BUFFER_SIZE = 1024
MAX_CLIENTS = 1
INITIAL_MESSAGE = 'Hello World'
INITIAL_FLAG = False

class Server():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((TCP_IP, TCP_PORT))
        self.sock.listen(MAX_CLIENTS)
        self.wait_for_connection()

    def wait_for_connection(self):
        self.client, self.addr = (self.sock.accept())
        data = self.client.recv(BUFFER_SIZE)
        print data
        if data == INITIAL_MESSAGE:
            print 'Received a handshake'
            global INITIAL_FLAG
            INITIAL_FLAG = True
            self.client.send(data)
        self.run()

    def run(self):
        while INITIAL_FLAG:
            data = self.client.recv(BUFFER_SIZE)
            print 'Data Received: ', data
            self.client.send(data)

print '--------- Server ---------'
server = Server()
