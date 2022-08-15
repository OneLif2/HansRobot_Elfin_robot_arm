from robot_arm import *
import time

def main():
    try:
        ra = RobotArm('192.168.1.111')
    except:
        pass
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


    ra.moveJoint(lpos2)
    ra2.moveJoint(rpos2)
    while ra.isMoving(): continue
    while ra2.isMoving(): continue

    for i in range(1, 6):
        ra.setSpeed(0.3) # i*0.2
        print(str(i)+')', 'speed', '=', ra.getSpeed())
        ra2.setSpeed(0.3) # i*0.2
        print(str(i)+')', 'speed', '=', ra2.getSpeed())
        
        ra.moveJoint(lpos1)
        ra2.moveJoint(rpos1)
        while ra.isMoving(): continue
        while ra2.isMoving(): continue

        ra.moveGripper(0)
        ra2.moveGripper(0)
        while ra.isGripperMoving(): continue
        while ra2.isGripperMoving(): continue
        ra.moveGripper(140)
        ra2.moveGripper(140)
        while ra.isGripperMoving(): continue
        while ra2.isGripperMoving(): continue
        ra.moveGripper(0)
        ra2.moveGripper(0)
        while ra.isGripperMoving(): continue
        while ra2.isGripperMoving(): continue

        ra.moveJoint(lpos2)
        ra2.moveJoint(rpos2)
        while ra.isMoving(): continue
        while ra2.isMoving(): continue

        ra.moveGripper(140)
        ra2.moveGripper(140)
        while ra.isGripperMoving(): continue
        while ra2.isGripperMoving(): continue
        ra.moveGripper(0)
        ra2.moveGripper(0)
        while ra.isGripperMoving(): continue
        while ra2.isGripperMoving(): continue

    print('Done. Resetting speed and position')
    ra.setSpeed(0.3)
    ra2.setSpeed(0.3)
    ra.moveGripper(140)
    ra2.moveGripper(140)
    ra.moveJoint(lhomepos2)
    ra2.moveJoint(rhomepos2)
    while ra.isMoving(): continue
    while ra2.isMoving(): continue
    ra.setSpeed(0.1)
    ra2.setSpeed(0.1)

if __name__ == '__main__':
    main()