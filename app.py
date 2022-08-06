from robot_arm import *
import time

def main():
    ra = RobotArm('192.168.1.111')
    pos1 = JPosition(-45.291, -13.130, 134.311, 105.14, -85.502, 180)
    pos2 = JPosition(0.994, -82.408, -83.538, -4.105, 86.534, 180)
    pos3 = JPosition(-33.250,-23.054,120.827,179.843,128.463,180)
    homepos = JPosition(-45.157,-12.859,148.379,-4.306,-129.731,180)

    # ra.setSpeed(0.3)
    # ra.moveJoint(pos2)
    while ra.isMoving(): continue
    for i in range(1, 6):
        ra.setSpeed(i * 0.2)
        print(str(i)+')', 'speed', '=', ra.getSpeed())
        
        ra.moveJoint(pos1)
        while ra.isMoving(): continue
        ra.moveGripper(140)
        time.sleep(0.8)
        ra.moveGripper(0)
        time.sleep(0.8)

        ra.moveJoint(pos2)
        while ra.isMoving(): continue
        ra.moveGripper(140)
        time.sleep(0.8)
        ra.moveGripper(0)
        time.sleep(0.8)
    print('Done. Resetting speed and position')
    ra.setSpeed(0.3)
    ra.moveGripper(0)
    ra.moveJoint(homepos)
    while ra.isMoving(): continue
    ra.setSpeed(0.1)

if __name__ == '__main__':
    main()