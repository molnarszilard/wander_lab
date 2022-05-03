import sys
sys.path.append("/home/jetson/")


import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import qwiic_scmd

import rospy
from sensor_msgs.msg import LaserScan

mindistance = 0.5

class Robot2(SingletonConfigurable):

    def __init__(self, *args, **kwargs):
        self.R_MTR = 0
        self.L_MTR = 1
        self.FWD = 0
        self.BWD = 1
        self.myMotor = qwiic_scmd.QwiicScmd()
        if self.myMotor.connected == False:
            print("Motor Driver not connected. Check connections.", \
                file=sys.stderr)
            return
        self.myMotor.begin()
        print("Motor initialized.")
        time.sleep(.250)
        # Zero Motor Speeds
        self.myMotor.set_drive(0,0,0)
        self.myMotor.set_drive(1,0,0)

        self.myMotor.enable()
        print("Motor enabled")
        time.sleep(.250)
        
    def set_motors(self, left_speed, right_speed):
        if left_speed<2.1:
            left_speed*=125
        if left_speed>255:
            left_speed=255
        if right_speed<2.1:
            right_speed*=125
        if right_speed>255:
            right_speed=255
        self.myMotor.set_drive(self.R_MTR,self.FWD,right_speed*3)
        self.myMotor.set_drive(self.L_MTR,self.FWD,left_speed*3)
        time.sleep(.1)
        
    def forward(self, speed=1.0, duration=None):
        if speed<2.1:
            speed*=125
        if speed>255:
            speed=255
        self.myMotor.set_drive(self.R_MTR,self.FWD,speed)
        self.myMotor.set_drive(self.L_MTR,self.FWD,speed)

    def backward(self, speed=1.0):
        if speed<2.1:
            speed*=125
        if speed>255:
            speed=255
        self.myMotor.set_drive(self.R_MTR,self.BWD,speed)
        self.myMotor.set_drive(self.L_MTR,self.BWD,speed)

    def left(self, speed=1.0):
        if speed<2.1:
            speed*=125
        if speed>255:
            speed=255
        self.myMotor.set_drive(self.R_MTR,self.FWD,speed)
        self.myMotor.set_drive(self.L_MTR,self.BWD,speed)

    def right(self, speed=1.0):
        if speed<2.1:
            speed*=125
        if speed>255:
            speed=255
        self.myMotor.set_drive(self.R_MTR,self.BWD,speed)
        self.myMotor.set_drive(self.L_MTR,self.FWD,speed)

    def stop(self):
        speed=0
        self.myMotor.set_drive(self.R_MTR,self.BWD,speed)
        self.myMotor.set_drive(self.L_MTR,self.FWD,speed)
        
    def test(self):
#         while True:
        speed = 20
        for speed in range(20,255):
            print(speed)
            self.myMotor.set_drive(self.R_MTR,self.FWD,speed)
            self.myMotor.set_drive(self.L_MTR,self.FWD,speed)
            time.sleep(.05)
        for speed in range(254,20, -1):
            print(speed)
            self.myMotor.set_drive(self.R_MTR,self.FWD,speed)
            self.myMotor.set_drive(self.L_MTR,self.FWD,speed)
            time.sleep(.05)
            
    def disable(self):
        self.stop()
        print("Ending example.")
        self.myMotor.disable()

robot = Robot2()

def fwd(a):
    robot.myMotor.set_drive(robot.L_MTR,robot.FWD,a*255)
    robot.myMotor.set_drive(robot.R_MTR,robot.FWD,a*255)

def stop(b):
    robot.myMotor.set_drive(robot.L_MTR,robot.FWD,0)
    robot.myMotor.set_drive(robot.R_MTR,robot.FWD,0)

def right(a):
    robot.myMotor.set_drive(robot.L_MTR,robot.FWD,a*255)
    robot.myMotor.set_drive(robot.R_MTR,robot.FWD,0)
    
def left(a):
    robot.myMotor.set_drive(robot.L_MTR,robot.FWD,0)
    robot.myMotor.set_drive(robot.R_MTR,robot.FWD,a*255)
    
def bck(a):
    robot.myMotor.set_drive(robot.L_MTR,robot.FWD,-a*255)
    robot.myMotor.set_drive(robot.R_MTR,robot.FWD,-a*255)

# robot.myMotor.set_drive(robot.L_MTR,robot.FWD,125)
# robot.myMotor.set_drive(robot.R_MTR,robot.FWD,125)
# time.sleep(5)
# robot.myMotor.set_drive(robot.L_MTR,robot.FWD,0)
# robot.myMotor.set_drive(robot.R_MTR,robot.FWD,0)

def callback(msg):
    stuck = False
    for i in range(msg.ranges.size()):
        if i>msg.ranges.size()*0.4 and i<msg.ranges.size()*0.6:
            if msg.ranges[i]<mindistance:
                stuck = True
    if stuck:
        right(0.5)
    else:
        fwd(0.5)
    print len(msg.ranges)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/pico_scan', LaserScan, callback)
rospy.spin()