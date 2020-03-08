import wpilib
from wpilib import controller as controller
import math
import rev
from networktables import NetworkTables
import logging
import sys
import time
import threading



class Turret:
	def __init__(self):
	
		self.revError = 30
		self.turretTolerance = .5
		
		self.turretMotorID = 15
		self.turretMotor = rev.CANSparkMax(self.turretMotorID,rev.MotorType.kBrushless)
		
		self.turretEncoder = self.turretMotor.getEncoder()
		self.turretEncoder.setPositionConversionFactor(12.414)
		self.spinEncoder = self.spinMotor.getEncoder()
		
		self.kP = 0.0085
		self.kI = 0.0
		self.kD = 0.0
		
		self.turretTurnController = wpilib.controller.PIDController(self.kP,self.kI,self.kD)
		self.turretTurnController.setSetpoint(0)
		self.turretTurnController.setTolerance(0.1)
		
		self.zeroTurret()
	def zeroTurret(self):
		self.turretEncoder.setPosition(0)
		
	def turretAlign(self,yaw,velocity):
		#assume negative moves negatively and pos moves positively
		turretOutput = self.turretTurnController.calculate(yaw)
		position = self.turretEncoder.getPosition()
		
		if self.turretMin < position < self.turretMax:
			self.turretMotor.set(-1*turretOutput)
			
		elif self.turretMin > position and turretOutput <0:
			self.turretMotor.set(-1*turretOutput)
			
		elif self.turretMax < position and turretOutput >0:
			self.turretMotor.set(-1*turretOutput)
		else:
			print('we like to not move')
			self.turretMotor.set(0)
		if (abs(yaw) < self.turretTolerance) and (abs(velocity-Turret.flywheelController.getEncoder().getVelocity()) <self.revError):
			print('SHOOT!!!!!!!!!!!!!!!!!!!')
			
	def stopTurret(self):
		self.turretMotor.set(0)