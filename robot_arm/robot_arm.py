from .tcp import Tcp
from .ra_error import *

class RobotArm():
    def __init__(self, host):
        self.tcp = Tcp(host)
    
    def _validate(self, reply):
        values = reply.split(',')
        if len(values) < 3:
            raise RAError('Received invalid response (' + reply + ')')
        if values[1] != 'OK':
            raise RAError('Received invalid response (' + reply + ')')

    def isMoving(self):
        reply = self.tcp.send('ReadMoveState,0,;')
        self._validate(reply)
        if reply == 'ReadMoveState,OK,1009,;':
            return True
        if reply == 'ReadMoveState,OK,0,;':
            return False
        raise RAError()

    def moveJoint(self, jPos):
        msg = 'MoveJ,0,' \
            + str(jPos.j1) + ',' + str(jPos.j2) + ',' \
            + str(jPos.j3) + ',' + str(jPos.j4) + ',' \
            + str(jPos.j5) + ',' + str(jPos.j6) + ',;'
        reply = self.tcp.send(msg)
        self._validate(reply)
    
    def setSpeed(
        self,
        speed: float
    ):
        msg = 'SetOverride,0,' \
            + str(speed) + ',;'
        reply = self.tcp.send(msg)
        self._validate(reply)

    def getSpeed(
        self
    ) -> float:
        msg = 'ReadOverride,0,;'
        reply = self.tcp.send(msg)
        self._validate(reply)
        values = reply.split(',')
        if len(values) < 4:
            raise RAError('Received invalid response.')
        return values[2]
        
    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def moveGripper(self, position, speed=250, force=10):
        self.clamp(position,0,140)
        self.clamp(speed,30,250)
        self.clamp(force,10,125)
        msg = 'SetRobotiq,' + str(position) + ',' + str(speed) + ',' + str(force) + ',;'
        reply = self.tcp.send(msg)
        self._validate(reply)
    
    def resetGripper(self):
        msg = 'RobotIQReset,;'
        reply = self.tcp.send(msg)
        self._validate(reply)
        print(reply)

    def isGripperMoving(self):
        reply = self.tcp.send('RobotiqStatus,;')
        self._validate(reply)
        if reply == 'RobotiqStatus,OK,0,3,1,1,;':
            return True #return True while its moving
        if reply == 'RobotiqStatus,OK,3,3,1,1,;':
            return False #return False when the action is done
        raise RAError()
