import socket

class Tcp:
    def __init__(self, host):
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.connect((host, 10003))
        
    def send(self, msg):
        self.s.send(str.encode(msg))
        return self.s.recv(1024).decode('UTF-8')
        
        