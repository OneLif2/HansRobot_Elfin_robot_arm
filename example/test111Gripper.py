# program for testing grippper status
import os
import sys
# Append parent directory to import path, import file from parent directory
sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(__file__) #./file.py
print(os.path.abspath(__file__)) #./file.py
print(os.path.dirname(os.path.abspath(__file__))) #./
print(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #../

from robot_arm import *
import time

def initial_setup():
    lpos1 = JPosition(-45.291, -13.130, 134.311, 105.14, -85.502, 0)
    lpos2 = JPosition(0.994, -82.408, -83.538, -4.105, 86.534, 0)
    lhomepos = JPosition(-45.157,-12.859,148.379,-4.306,-129.731,0)
    lhomepos2 = JPosition(-35,-16.774,140,180,130,0)

    try:
        ra2 = RobotArm('192.168.1.112')
    except:
        pass
    rpos1 = JPosition(-45.291, -13.130, 134.311, 105.14, -85.502, 0)
    rpos2 = JPosition(0.994, -82.408, -83.538, -4.105, 86.534, 0)
    rhomepos = JPosition(-45.157,-12.859,148.379,-4.306,-129.731,0)
    rhomepos2 = JPosition(-35,-16.774,140,0,-130,0)

def main():
    try:
        ra = RobotArm('192.168.1.112')
    except:
        pass
    print("ready!!")

    #ra.resetGripper()

    for i in range(1, 6):
        ra.moveGripper(140,200,10)

        start_time = time.time()
        time_diff = 0
        while time_diff < 5 :
            reply = ra.tcp.send("RobotiqStatus,;")
            print(reply)
            time_diff = time.time() - start_time
        print("action 1 done!!")
        time.sleep(1)
        
        ra.moveGripper(82,200,10)

        start_time = time.time()
        time_diff = 0
        while time_diff < 5 :
            reply = ra.tcp.send("RobotiqStatus,;")
            print(reply)
            time_diff = time.time() - start_time
        print("action 2 done!!")
        time.sleep(1)

if __name__ == '__main__':
    initial_setup()
    main()