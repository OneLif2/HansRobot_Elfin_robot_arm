import socket  # sudo apt-get install socket
from threading import Thread
import time

host = "192.168.1.111"  # server ip
port = 10003

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

is_first_input = True
quit_prog2 = False

s.send(str.encode("GetVersion,;"))
print("return = ", s.recv(1024).decode('UTF-8'))
print("\n")

def prog1():

    while True:
        global is_new_line, quit_prog2
        if is_new_line == True:
            command = input('Enter your command: ') + ",;\n"
            is_new_line = False
        else:
            command = input() + ",;\n"

        if command == 'EXIT':
            # Send EXIT request to other end
            s.send(str.encode(command))
            break
        elif command == 'Quit':
            # Send KILL command
            s.send(str.encode(command))
            quit_prog2 == True
            break
        else:
            s.send(str.encode(command))
    quit_prog2 == True
    s.close()

def prog2():
    while True:
        reply = s.recv(1024)
        print(reply.decode('UTF-8'))
        print("")
        print('Enter your command: ', end=' ')
        time.sleep(0.5)

thread1 = Thread(target=prog1)
thread2 = Thread(target=prog2)
thread1.start()
thread2.start()
