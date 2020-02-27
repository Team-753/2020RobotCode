#!/usr/bin/env python3

import wpilib
import math
import rev
from wpilib import controller as controller
from navx import AHRS
from networktables import NetworkTables
import logging
import sys
import time
import threading
from AutoTurret import Turret

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

# To see messages from networktables, you must setup logging 
NetworkTables.initialize() 
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

sd = NetworkTables.getTable('chameleon-vision').getSubTable('Microsoft LifeCam HD-3000')

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.dNaught = 18.0416
		self.aNaught = 5946
		self.joy = wpilib.Joystick(0)
		self.turret = Turret()
		self.visionTime = False
		
	def autonomousInit(self):
		self.turret.zeroTurret()
		print('autonomous started')
		
	def autonomousPeriodic(self):
		self.turret.checkEncoders()
		
	def teleopInit(self):
		print('teleop started')
		
	def teleopPeriodic(self):
		if self.joy.getRawButton(2):
			print('test')
		
		self.area = (sd.getEntry('targetFittedWidth').getDouble(0))*(sd.getEntry('targetFittedHeight').getDouble(0))
		try :
			self.distance = self.aNaught/self.area
			
		except:
			print("no area")
			self.distance = 1
		
		print('the yaw is '+ str(sd.getEntry('targetYaw').getDouble(0)))
		print('the pitch is ' + str(sd.getEntry('targetPitch').getDouble(0)))
		#print('the distance is ' + str(self.distance*self.dNaught))
		
		#print('turret position at ' + str(360*(wpilib.AnalogInput(4).getValue()/2522)))
		

		if self.joy.getRawButton(7):
			print('vision time')
			self.visionTime = True
		if self.joy.getRawButton(8):
			self.visionTime = False
			
		if self.visionTime:
			self.turret.turretAlign(sd.getEntry('targetYaw').getDouble(0))
		else:
			self.turret.stopTurret()

if __name__ == "__main__":
	wpilib.run(MyRobot)