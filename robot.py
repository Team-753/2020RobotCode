#!/usr/bin/env python3

import wpilib
import math
import time
from DriveTrain import DriveTrain
from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from Turret import Turret
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.navx = AHRS.create_spi()
		wpilib.CameraServer.launch()
		
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
		self.scaling = .8
		self.intakeSpeed = .35
		self.halfMoonSpeed = .8
		self.feederSpeed = 10 #volts
		self.turretSpeed = .085
		self.climbExtend = .35
		self.climbContract = .75
		self.manualFlywheelSpeed = 8.5 #volts
		
		self.navx.reset()
		
	def checkDeadband(self, axis, check):
		if check:
			deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", self.joystickDeadband)
			if abs(axis) < deadband:
				axis = 0
		else:
			deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", .4)
			if abs(axis) < deadband:
				axis = 0
		
		#this fixes the speed to start at 0 for the deadband of the axis and still make it to 1 when the axis is max
		if axis > 0:
			axis -= deadband
		if axis < 0:
			axis += deadband
		axis *= 1/(1-deadband)
		
		return axis
		
	def checkSwitches(self):
		#buttons list
		intakeIn = self.joystick.getRawButton(2)
		intakeOut = self.auxiliary.getXButton()
		climbUp = self.auxiliary.getYButton()
		climbDown = self.auxiliary.getAButton()
		turretClockwise = self.auxiliary.getBumper(wpilib.interfaces.GenericHID.Hand.kRightHand)
		turretCounterclockwise = self.auxiliary(wpilib.interfaces.GenericHID.Hand.kLeftHand)
		autoAim = False #trigger stuff
		manualFlywheel = self.auxiliary.getStickButton(wpilib.interfaces.GenericHID.Hanf.kLeftHand)
		feederIn = self.auxiliary.getBButton()
		
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
			#auto aim code
			pass
		else:
			#manual turret rotation
			if turretClockwise:
				self.turret.turretManual(-self.turretSpeed) #turret speed percent
			elif turretCounterclockwise:
				self.turret.turretManual(self.turretSpeed) #turret speed percent
			else:
				self.turret.turretManual(0)
			
			#manual flywheel
			if manualFlywheel:
				self.turret.flywheelManual(flywheelSpeed) #voltage
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
		pass
		
	def teleopInit(self):
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('teleop started')
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		x = self.scaling*self.checkDeadband(self.joystick.getX(),True)
		y = -self.scaling*self.checkDeadband(self.joystick.getY(),True)
		z = self.scaling*self.checkDeadband(self.joystick.getZ(),False)
		
		if self.joystick.getRawButton(7):
			x *= .25
			y *= .25
			z *= .25
		
		angle = -1*self.navx.getAngle() + 90
		angle *= math.pi/180
		
		if not self.joystick.getRawButton(1):
			cos = math.cos(angle)
			sin = math.sin(angle)
			temp = x*sin - y*cos
			y = x*cos + y*sin
			x = temp
		
		if max(abs(x),abs(y),abs(z)) != 0:
			self.drive.move(x,y,z)
		else:
			self.drive.stationary()
		
	def testInit(self):
		self.drive.coast()
		self.navx.reset()
		print("Starting tests")
		
	def testPeriodic(self):
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