#!/usr/bin/env python3

import wpilib
from wpilib import controller as controller
import math
from DriveTrain import DriveTrain
from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from Turret import Turret
from navx import AHRS
from networktables import NetworkTables
import threading

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
		self.navx = AHRS.create_spi()
		wpilib.CameraServer.launch()
		wpilib.DigitalOutput(0).set(0)
		
		self.joystick = wpilib.Joystick(0)
		self.auxiliary = wpilib.XboxController(1)
		
		self.drive = DriveTrain()
		self.climb = Climb(16) #climb ID
		self.feeder = Feeder(9) #feeder ID
		self.intake = Intake(11,10) #intake ID, half moon ID
		self.turret = Turret(15,14) #rotate ID, flywheel ID
		
		self.drive.zeroEncoders()
		self.climb.coast()
		self.intake.coast()
		
		#constants
		self.joystickDeadband = .2
		self.scaling = .55
		self.intakeSpeed = .35
		self.halfMoonSpeed = .8
		self.feederSpeed = .85
		self.turretSpeed = .2
		self.climbExtend = .35
		self.climbContract = .75
		self.manualFlywheelSpeed = 8.5 #volts
		
		self.navx.reset()
		
		self.kP = 0.0016
		self.kI = 0.0003
		self.kD = 0
		self.driveAngleController = wpilib.controller.PIDController(self.kP,self.kI,self.kD)
		self.driveAngleController.enableContinuousInput(-180,180)
		
		wpilib.SmartDashboard.putNumber("Shooter RPM",0)
		
	def checkDeadband(self, axis, check):
		if check:
			deadband = self.joystickDeadband
			if abs(axis) < deadband:
				axis = 0
		else:
			deadband = .4
			if abs(axis) < deadband:
				axis = 0
		
		#this fixes the speed to start at 0 for the deadband of the axis and still make it to 1 when the axis is max
		if axis > 0:
			axis -= deadband
		if axis < 0:
			axis += deadband
		axis *= 1/(1-deadband)
		
		return axis
		
	def navxBoundedPosition(self):
		angle = -1*self.navx.getAngle() + 90
		angle %= 360
		if angle < -180:
			angle += 360
		elif angle > 180:
			angle -= 360
		return(angle)
		
	def checkSwitches(self):
		#buttons list
		intakeIn = self.joystick.getRawButton(2)
		intakeOut = self.auxiliary.getXButton()
		climbUp = self.auxiliary.getYButton()
		climbDown = self.auxiliary.getAButton()
		turretClockwise = self.auxiliary.getStickButton(wpilib.interfaces.GenericHID.Hand.kRightHand)
		turretCounterclockwise = self.auxiliary.getStickButton(wpilib.interfaces.GenericHID.Hand.kLeftHand)
		autoAim = self.auxiliary.getBumper(wpilib.interfaces.GenericHID.Hand.kRightHand)
		manualFlywheel = self.auxiliary.getBumper(wpilib.interfaces.GenericHID.Hand.kLeftHand)
		feederIn = self.auxiliary.getBButton()
		
		manualFlywheelSpeed = wpilib.SmartDashboard.getNumber("Shooter RPM",4000)
		
		#intake control
		if intakeIn:
			self.intake.collect(self.intakeSpeed,self.halfMoonSpeed) #intake speed, half moon speed
		elif intakeOut:
			self.intake.expel(self.intakeSpeed,self.halfMoonSpeed) #intake speed, half moon speed
		else:
			self.intake.stop()
		
		#climb control
		if climbUp:
			self.climb.extend(self.climbExtend)
		elif climbDown:
			self.climb.contract(self.climbContract)
		else:
			self.climb.stop()
		
		if autoAim:
			wpilib.DigitalOutput(0).set(1)
			
			#,sd.getEntry('isValid').getBoolean()
			self.turret.aim(sd.getEntry('targetFittedHeight').getDouble(0),sd.getEntry('targetYaw').getDouble(0))
			wpilib.SmartDashboard.putNumber("rec height",sd.getEntry('targetFittedHeight').getDouble(0))
			
			if feederIn:
					self.feeder.feed(self.feederSpeed)
			else:
				self.feeder.stop()
		else:
			#manual turret rotation
			wpilib.DigitalOutput(0).set(0)
			if turretClockwise:
				self.turret.turretManual(self.turretSpeed) #turret speed percent
			elif turretCounterclockwise:
				self.turret.turretManual(-self.turretSpeed) #turret speed percent
			else:
				self.turret.turretManual(0)
			
			#manual flywheel
			if manualFlywheel:
				self.turret.flywheelRPM(manualFlywheelSpeed)
				if feederIn:
					self.feeder.feed(self.feederSpeed)
				else:
					self.feeder.stop()
			else:
				self.turret.flywheelManual(0)
				self.feeder.stop()
		
	def autonomousInit(self):
		self.navx.reset()
		self.drive.zeroEncoders()
		self.climb.zeroEncoder()
		self.feeder.zeroEncoder()
		self.navx.reset()
		
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('autonomous started')
		self.navx.reset()
		
	def autonomousPeriodic(self):
		#we should put something here at some point
		circumference = 4*math.pi
		goDistanceY = 36 #inches
		encoderPointsPerRev = 18
		revsForFeeder = 500
		
		x,y = self.drive.averageWheelPosition()
		if y < (goDistanceY/circumference)*encoderPointsPerRev:
			self.drive.move(0,.15,0)
		else:
			self.drive.stationary()
			if self.feeder.getPosition() < revsForFeeder*encoderPointsPerRev:
				wpilib.DigitalOutput(0).set(1)
				self.feeder.feed(self.feederSpeed)
				self.turret.aim(sd.getEntry('targetFittedHeight').getDouble(0),sd.getEntry('targetYaw').getDouble(0))
			else:
				wpilib.DigitalOutput(0).set(0)
				self.feeder.stop()
				self.turret.flywheelManual(0)
		
	def teleopInit(self):
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('teleop started')
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		ruben = self.turret.getFlywheelSpeed()
		wpilib.SmartDashboard.putNumber("Flywheel speed output",ruben)
		wpilib.SmartDashboard.putNumber("Flywheel speed graph",ruben)
		
		x = self.scaling*self.checkDeadband(self.joystick.getX(),True)
		y = -self.scaling*self.checkDeadband(self.joystick.getY(),True)
		z = .65*self.scaling*self.checkDeadband(self.joystick.getZ(),False)
		
		if self.joystick.getRawButton(7):
			x *= .3
			y *= .3
			z *= .3
		
		angleDegrees = self.navxBoundedPosition()
		angleRadians = angleDegrees*math.pi/180
		
		if not self.joystick.getRawButton(1):
			cos = math.cos(angleRadians)
			sin = math.sin(angleRadians)
			temp = x*sin - y*cos
			y = x*cos + y*sin
			x = temp
		
		if self.joystick.getRawButton(11):
			self.driveAngleController.setSetpoint(90)
			z = self.driveAngleController.calculate(angleDegrees)
		if self.joystick.getRawButton(12):
			self.driveAngleController.setSetpoint(-90)
			z = self.driveAngleController.calculate(angleDegrees)
		
		if max(abs(x),abs(y),abs(z)) != 0:
			self.drive.move(x,y,z)
		else:
			self.drive.stationary()
		
	def testInit(self):
		self.drive.coast()
		self.navx.reset()
		print("Starting tests")
		wpilib.DigitalOutput(0).set(1)
		
	def testPeriodic(self):
		print(str(sd.getEntry('targetFittedHeight').getDouble(0)))
		self.drive.checkEncoders()
		if self.auxiliary.getBumper(wpilib.interfaces.GenericHID.Hand.kRightHand):
			self.turret.turretManual(-self.turretSpeed)
		elif self.auxiliary.getBumper(wpilib.interfaces.GenericHID.Hand.kLeftHand):
			self.turret.turretManual(self.turretSpeed)
		else:
			self.turret.turretManual(0)
		
	def disabledInit(self):
		self.drive.coast()
		self.turret.coast()
		self.feeder.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)