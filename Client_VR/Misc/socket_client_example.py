import socket, time, os, random

import socket

TCP_IP = '128.46.125.46' # The static IP of Ubuntu computer
TCP_PORT = 5000 # Both server and client should have a common IP and Port
BUFFER_SIZE = 1024 # in bytes. 1 charecter is one byte.
INITIAL_MESSAGE = 'Handshake'
CONNECT_STATUS = False

class Client():
    def __init__(self):
        self.sock = socket.socket()
        self.sock.connect((TCP_IP, TCP_PORT))
        self.sock.send(INITIAL_MESSAGE)
        data = self.sock.recv(BUFFER_SIZE) # Blocking call
        if data == 'True':
            print('Handshake successfull ! ! !')
            global CONNECT_STATUS
            CONNECT_STATUS = True
        # def send_data()
        while CONNECT_STATUS:
            print self.sock.send(str(23.2568))
            print self.sock.recv(BUFFER_SIZE)
            # time.sleep(1)

    def connection_status(self):
        return CONNECT_STATUS

c = Client()
