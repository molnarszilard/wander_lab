import sys
sys.path.append("/home/jetson/")


import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import qwiic_scmd
import numpy
import rospy
from sensor_msgs.msg import LaserScan
import atexit

mindistance = 0.5

class Robot2(SingletonConfigurable):

    def __init__(self, *args, **kwargs):
        self.R_MTR = 0
        self.L_MTR = 1
        self.FWD = 0
        self.BWD = 1
        self.myMotor = qwiic_scmd.QwiicScmd()
        if self.myMotor.connected == False:
            print("Motor Driver not connected. Check connections.")
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
    #only_nan = True
    only_nan = False
    for i in range(len(msg.ranges)):
        if i>len(msg.ranges)*0.4 and i<len(msg.ranges)*0.6:
            if msg.ranges[i]<mindistance:
                stuck = True
        #if not numpy.isnan(msg.ranges[i]):
        #       print("only nan can found, i am stopping")
        #       only_nan = False
    if stuck:
        print("Obstacle ahead. I am turning right")
        right(0.5)
    else:
        fwd(0.35)
    if len(msg.ranges)<1 or only_nan:
        stop(0)
    
def my_exit_function(some_argument):
    stop(0)
    print(some_argument)

atexit.register(my_exit_function, 'Program closed. I am stopping.', )
rospy.init_node('scan_values')
sub = rospy.Subscriber('/pico_scan', LaserScan, callback)
rospy.spin()

