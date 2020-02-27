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
from AutoTurret import TurretAuto

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

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		
	def autonomousInit(self):
		self.instantiate.zeroTurret()
		
		print('autonomous started')
		
	def autonomousPeriodic(self):
		
		self.instantiate.checkEncoders()
		
	def teleopInit(self):
		
		print('teleop started')
		
	def teleopPeriodic(self):
		if wpilib.Joystick(0).getRawButton(8):
			Networktables.getTable('SmartDashboard').putBoolean('Light',True)
		else:
			Networktables.getTable('SmartDashboard').putBoolean('Light',False)

if __name__ == "__main__":
	wpilib.run(MyRobot)