#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Bool
from PCA9685 import PCA9685

import RPi.GPIO as GPIO
import time
Ab_obj=None
class AlphaBot2(object):
    def __init__(self,OBS_L=19,OBS_R=16):
        self.obs_left=OBS_L
        self.obs_right=OBS_R
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.obs_left,GPIO.IN,GPIO.PUD_UP)
        GPIO.setup(self.obs_right,GPIO.IN,GPIO.PUD_UP)
    def get_front_obs_sensor_reading(self):
        DR_status = GPIO.input(self.obs_left)
        DL_status = GPIO.input(self.obs_right)
        return 1-DL_status,1-DR_status
def sensor_transmitter():
    global Ab_obj
    Ab_obj=AlphaBot2()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_sensor_transmitter', anonymous=True)
    pub_obs_left = rospy.Publisher('robot_sensor/front_obs/left', String, queue_size=3)
    pub_obs_right = rospy.Publisher('robot_sensor/front_obs/right', String, queue_size=3)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_obs,right_obs=Ab_obj.get_front_obs_sensor_reading()
        pub_obs_left.publish(str(left_obs))
        pub_obs_right.publish(str(right_obs))
        rate.sleep()
    
if __name__ == '__main__':
    sensor_transmitter()
