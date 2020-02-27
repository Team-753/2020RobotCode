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
		self.maxTurretSpeed = .2
		
		self.flywheelPort = 0
		#determine from Chazzy-poo
		self.hoodReduction = 12
		
		self.turretMotorID = 15
		self.spinMotorID = 14
		self.engageMotorID = 9
		
		self.absTurretEncoder = wpilib.AnalogInput(4)
		
		self.engageMotor = rev.CANSparkMax(self.engageMotorID,rev.MotorType.kBrushless)
		self.turretMotor = rev.CANSparkMax(self.turretMotorID,rev.MotorType.kBrushless)
		self.spinMotor = rev.CANSparkMax(self.spinMotorID,rev.MotorType.kBrushless)
		
		self.turretEncoder = self.turretMotor.getEncoder()
		self.turretEncoder.setPositionConversionFactor(12.414)
		self.spinEncoder = self.spinMotor.getEncoder()
		
		self.kP = 0.005
		self.kI = 0.0
		self.kD = 0.0
		
		self.turretTurnController = wpilib.controller.PIDController(self.kP,self.kI,self.kD)
		self.turretTurnController.setSetpoint(0)
		self.turretTurnController.setTolerance(0.1)
		
		self.turretMin = -80
		self.turretMax = 80
		
		self.sP = 0.0
		self.sI = 0.0
		self.sD = 0.0
		self.cruisingVelocity = 5600 #in rpm
		self.minVelocity = 1000
		
		self.spinController = self.spinMotor.getPIDController()
		self.spinController.setP(self.sP,self.flywheelPort)
		self.spinController.setI(self.sI,self.flywheelPort)
		self.spinController.setD(self.sD,self.flywheelPort)
		self.spinController.setSmartMotionMaxVelocity(self.cruisingVelocity,self.flywheelPort)
		self.spinController.setSmartMotionMinOutputVelocity(self.minVelocity, self.flywheelPort)
		
		#don't know whether we are using two motors, likely depends on an epic battle between Liam and Chaz
		#Chaz won for now (we are just using one)
		self.stallLimit = 78
		self.freeLimit  = 22
		self.limitRPM  = 2200
		self.spinMotor.setSmartCurrentLimit(self.stallLimit,self.freeLimit,self.limitRPM)
		
	def checkEncoders(self):
		absPosition = 360*self.absTurretEncoder.getValue()/2522
		position = self.turretEncoder.getPosition()
		print("Absolute says" + str(absPosition))
		print("Normal is" + str(position))
		
	def zeroTurret(self):
		absEncoderCorrected = (360*(self.absTurretEncoder.getValue()/2522))
		self.turretEncoder.setPosition(absEncoderCorrected -320)
		
	def velocityControl(self,desiredVelocity):
		self.spinController.setReference(desiredVelocity,rev.ControlType.kSmartVelocity,self.flywheelPort)
		
	def turretAlign(self,yaw):
		#assume negative moves negatively and pos moves positively
		turretOutput = self.turretTurnController.calculate(yaw)
		print('turret alignment engaged, moving at ' +str(turretOutput))
		
		position = self.turretEncoder.getPosition()
		print('the turret is at ' + str(position))
		
			#clockwise is negative
		if turretOutput < -self.turretMaxSpeed:
			self.turretOutput = -self.turretMaxSpeed
		elif turretOutput > self.turretMaxSpeed:
			self.turretOutput = self.turretMaxSpeed
		
		if self.turretMin < position < self.turretMax:
			self.turretMotor.set(turretOutput)
		else:
			self.turretMotor.set(0)
		
	def turretPretendAlign(self,yaw):
		turretOutput = self.turretTurnController.calculate(yaw)
		print('turret alignment engaged, moving at ' +str(turretOutput))
		
		position = self.turretEncoder.getPosition()
		print('the turret is at ' + str(position))
		
			#clockwise is negative
		if turretOutput < -self.turretMaxSpeed:
			self.turretOutput = -self.turretMaxSpeed
		elif turretOutput > self.turretMaxSpeed:
			self.turretOutput = self.turretMaxSpeed
		
	def stopTurret(self):
		self.turretMotor.set(0)
		
	def turretShoot(self,yaw):
		#the turret must first align with the plane of the center of the target
		self.turretAlign(yaw)
	
		#as of current, we are maintainin a constant angle
		#self.velocityControl(vDesired)
		'''
		if (vDesired > self.maxVelocity) or (vDesired < self.minVelocity):
			#Angular control
			
			angularControl()
		else:
			#velocity control
			self.velocityControl(vDesired)
			'''
		if self.shootButton:
			self.engageMotor.set(0.7) #not a confirmed speed
		else:
			self.engageMotor.set(0)
		
	def brake(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kBrake)
		self.spinMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def coast(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kCoast)
		self.spinMotor.setIdleMode(rev.IdleMode.kCoast)
		