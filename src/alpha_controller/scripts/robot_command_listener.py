#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from PCA9685 import PCA9685

import RPi.GPIO as GPIO
import time
Ab_obj=None
class AlphaBot2(object):
	
	def __init__(self,ain1=12,ain2=13,ena=6,bin1=20,bin2=21,enb=26):
		self.AIN1 = ain1
		self.AIN2 = ain2
		self.BIN1 = bin1
		self.BIN2 = bin2
		self.ENA = ena
		self.ENB = enb
		self.PA  = 50
		self.PB  = 50
		self.Servo1_channel=1
		self.Servo2_channel=0
		self.pan_pulse=1500
		self.tilt_pulse=1850
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.AIN1,GPIO.OUT)
		GPIO.setup(self.AIN2,GPIO.OUT)
		GPIO.setup(self.BIN1,GPIO.OUT)
		GPIO.setup(self.BIN2,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.PWMA = GPIO.PWM(self.ENA,500)
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(self.PA)
		self.PWMB.start(self.PB)
		self.pwm_9685 = PCA9685(0x40)
		self.pwm_9685.setPWMFreq(50)
		self.pwm_9685.setServoPulse(self.Servo1_channel,self.tilt_pulse)
		self.pwm_9685.setServoPulse(self.Servo2_channel,self.pan_pulse)
		self.stop()

	def forward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.AIN1,GPIO.LOW)
		GPIO.output(self.AIN2,GPIO.HIGH)
		GPIO.output(self.BIN1,GPIO.LOW)
		GPIO.output(self.BIN2,GPIO.HIGH)


	def stop(self):
		self.PWMA.ChangeDutyCycle(0)
		self.PWMB.ChangeDutyCycle(0)
		GPIO.output(self.AIN1,GPIO.LOW)
		GPIO.output(self.AIN2,GPIO.LOW)
		GPIO.output(self.BIN1,GPIO.LOW)
		GPIO.output(self.BIN2,GPIO.LOW)

	def backward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.AIN1,GPIO.HIGH)
		GPIO.output(self.AIN2,GPIO.LOW)
		GPIO.output(self.BIN1,GPIO.HIGH)
		GPIO.output(self.BIN2,GPIO.LOW)

		
	def left(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.AIN1,GPIO.HIGH)
		GPIO.output(self.AIN2,GPIO.LOW)
		GPIO.output(self.BIN1,GPIO.LOW)
		GPIO.output(self.BIN2,GPIO.HIGH)


	def right(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.AIN1,GPIO.LOW)
		GPIO.output(self.AIN2,GPIO.HIGH)
		GPIO.output(self.BIN1,GPIO.HIGH)
		GPIO.output(self.BIN2,GPIO.LOW)
		
	def setPWMA(self,value):
		self.PA = value
		self.PWMA.ChangeDutyCycle(self.PA)

	def setPWMB(self,value):
		self.PB = value
		self.PWMB.ChangeDutyCycle(self.PB)	
		
	def setMotor(self, left, right):
		if((right >= 0) and (right <= 100)):
			GPIO.output(self.AIN1,GPIO.HIGH)
			GPIO.output(self.AIN2,GPIO.LOW)
			self.PWMA.ChangeDutyCycle(right)
		elif((right < 0) and (right >= -100)):
			GPIO.output(self.AIN1,GPIO.LOW)
			GPIO.output(self.AIN2,GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - right)
		if((left >= 0) and (left <= 100)):
			GPIO.output(self.BIN1,GPIO.HIGH)
			GPIO.output(self.BIN2,GPIO.LOW)
			self.PWMB.ChangeDutyCycle(left)
		elif((left < 0) and (left >= -100)):
			GPIO.output(self.BIN1,GPIO.LOW)
			GPIO.output(self.BIN2,GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - left)

	def pan(self,direction):
		self.pan_pulse+=direction*50
		if self.pan_pulse<500:
			self.pan_pulse=500
		if self.pan_pulse>3000:
			self.pan_pulse=3000
		print(self.pan_pulse)
		self.pwm_9685.setServoPulse(self.Servo2_channel,self.pan_pulse)
	def tilt(self,direction):
		self.tilt_pulse+=direction*50
		if self.tilt_pulse<800:
			self.tilt_pulse=800
		if self.tilt_pulse>2500:
			self.tilt_pulse=2500
		print(self.tilt_pulse)
		self.pwm_9685.setServoPulse(self.Servo1_channel,self.tilt_pulse)
def callback(data):
    print(data.data)
    msg=data.data   
    if msg=="w":
        Ab_obj.forward()
    elif msg=="s":
        Ab_obj.backward()
    elif msg=="a":
        Ab_obj.left()
    elif msg=="d":
        Ab_obj.right()
    elif msg=="x":
        Ab_obj.stop()
    elif msg=="i":
		Ab_obj.tilt(-1)
    elif msg=="k":
		Ab_obj.tilt(1)
    elif msg=="j":
		Ab_obj.pan(1)
    elif msg=="l":
		Ab_obj.pan(-1)
	
def listener():
    global Ab_obj
    Ab_obj=AlphaBot2()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_command_listener', anonymous=True)

    rospy.Subscriber("robot_commands", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
