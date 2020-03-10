import math
import rev
import wpilib
from networktables import NetworkTables
from wpilib import controller as controller

class SwerveModule:
	wheelDiameter = 4 #inches
	turnMotorEncoderConversion = 20 #NEO encoder gives 0-18 as 1 full rotation
	absoluteEncoderConversion = .08877
	
	def __init__(self,driveID,turnID,encoderID,encoderOffset,name):
		if name == "Front Left":
			kPTurn = .006
			kITurn = .002
		elif name == "Front Right":
			kPTurn = .008
			kITurn = .002
		elif name == "Rear Left":
			kPTurn = .007
			kITurn = .002
		elif name == "Rear Right":
			kPTurn = .0066
			kITurn = 0
		else:
			kPTurn = .0066
			kITurn = .0018
		kDTurn = 0
		
		self.driveMotor = rev.CANSparkMax(driveID,rev.MotorType.kBrushless)
		self.driveMotor.restoreFactoryDefaults()
		self.turnMotor = rev.CANSparkMax(turnID,rev.MotorType.kBrushless)
		
		self.turnEncoder = self.turnMotor.getEncoder()
		self.turnEncoder.setPositionConversionFactor(self.turnMotorEncoderConversion) #now is 0-360
		
		self.driveEncoder = self.driveMotor.getEncoder()
		
		self.absoluteEncoder = wpilib.AnalogInput(encoderID)
		self.encoderOffset = encoderOffset
		
		self.turnController = wpilib.controller.PIDController(kPTurn, kITurn, kDTurn)
		self.turnController.enableContinuousInput(-180,180) #the angle range we decided to make standard
		
		self.turnDeadband = .035 #this needs to be updated
		self.moduleName = name
		
		self.x = 0
		self.y = 0
		self.pastPosition = 0
		
	def encoderBoundedPosition(self):
		position = self.turnEncoder.getPosition()%360 #this limits the encoder input
		if position < 0: #to be on a single circle
			position += 360
		if position < 90: #this translates those values to correspond with what the
			position += 90 #atan2 function returns (-180, 180)
		else:
			position -= 270
		return position
		
	def move(self,driveSpeed,angle):
		position = self.encoderBoundedPosition()
		
		self.turnController.setSetpoint(angle) #tells the PID controller what our goal is
		turnSpeed = self.turnController.calculate(position) #gets the ideal speed from the PID controller
		
		if abs(turnSpeed) < self.turnDeadband:
			turnSpeed = 0
		
		self.driveMotor.set(driveSpeed)
		self.turnMotor.set(turnSpeed)
		
		#wpilib.SmartDashboard.putNumber(self.moduleName,position)
		
		'''wpilib.SmartDashboard.putNumber(self.moduleName + " Position",position)
		badVelocity = self.driveEncoder.getVelocity()
		wpilib.SmartDashboard.putNumber(self.moduleName + " Velocity",badVelocity)
		'''
	def stationary(self):
		self.driveMotor.set(0) #this will be smoother once we drive with velocity PID (by setting setpoint to 0)
		
		position = self.encoderBoundedPosition()
		turnSpeed = self.turnController.calculate(position)
		
		if abs(turnSpeed) < self.turnDeadband:
			turnSpeed = 0
		
		self.turnMotor.set(turnSpeed) #just let the turn motor go to its most recent goal
		'''wpilib.SmartDashboard.putNumber(self.moduleName + " Position",position)
		badVelocity = self.driveEncoder.getVelocity()
		wpilib.SmartDashboard.putNumber(self.moduleName + " Velocity",badVelocity)'''
		
	def returnAbsolutes(self):
		position = self.absoluteEncoder.getValue()*self.absoluteEncoderConversion
		return(position)
		
	def zeroEncoder(self,absolutePosition):
		sparkPosition = 360 - absolutePosition + self.encoderOffset
		
		self.driveEncoder.setPosition(0)
		self.turnEncoder.setPosition(sparkPosition)
		self.turnController.setSetpoint(0)
		
	def brake(self):
		self.driveMotor.setIdleMode(rev.IdleMode.kBrake)
		self.turnMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def coast(self):
		self.driveMotor.setIdleMode(rev.IdleMode.kCoast)
		self.turnMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def stopTurn(self):
		self.turnMotor.set(0)
		
	def checkEncoders(self):
		absolutePosition = self.absoluteEncoder.getValue()*self.absoluteEncoderConversion
		position = self.encoderBoundedPosition()
		wpilib.SmartDashboard.putNumber(self.moduleName,absolutePosition)
		wpilib.SmartDashboard.putNumber(self.moduleName + " NEO",position)
		
	def autoPosition(self):
		currentPosition = self.driveEncoder.getPosition()
		difference = currentPosition - self.pastPosition
		angle = self.encoderBoundedPosition()*math.pi/180
		self.x += difference*math.cos(angle)
		self.y += difference*math.sin(angle)
		self.pastPosition = currentPosition
		return(self.x,self.y)
		