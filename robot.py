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
		
		wpilib.SmartDashboard.putNumber("percent shooot", .8)
		
		self.joystick = wpilib.Joystick(0)
		self.auxiliaryJoystick = wpilib.Joystick(1)
		
		
		self.drive = DriveTrain()
		self.climb = Climb(16) #climb ID
		self.feeder = Feeder(9) #feeder ID
		self.intake = Intake(11,10) #intake ID, half moon ID
		self.turret = Turret(15,14) #rotate ID, flywheel ID
		
		self.drive.zeroEncoders()
		self.climb.coast()
		self.intake.coast()
		
		self.dumbCounter = 0
		
		#constants
		self.joystickDeadband = .2
		self.scaling = .8
		self.intakeSpeed = .35
		self.halfMoonSpeed = .8
		self.feederSpeed = .8
		self.turretSpeed = .085
		self.climbSpeed = .35
		self.dumbAutoDistance = -25 #inches? ticks? meters? About 8 per foot, so...
		self.flywheelSpeed = .65
		
		#I hate this but I'm also lazy
		self.autoSpun = False
		self.dumbFeederDistance = 500 #how many ticks it reads in time to shoot 3 cells
		
		#put constants to shuffleboard
		'''wpilib.SmartDashboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putNumber("Intake speed",self.intakeSpeed)
		wpilib.SmartDashboard.putNumber("Half moon speed",self.halfMoonSpeed)
		wpilib.SmartDashboard.putNumber("Feeder speed",self.feederSpeed)
		wpilib.SmartDashboard.putNumber("Turret speed",self.turretSpeed)'''
		
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
		autoManualSwitch = self.auxiliaryJoystick.getRawButton(7)
		
		intakeIn = self.joystick.getRawButton(2)
		intakeOut = self.joystick.getRawButton(5)
		
		feederIn = self.joystick.getRawButton(11)
		
		manualFlywheel = self.joystick.getRawButton(12)
		turretClockwise = self.joystick.getRawButton(7)
		turretCounterclockwise = self.joystick.getRawButton(8)
		
		climbUp = self.joystick.getRawButton(4)
		climbDown = self.joystick.getRawButton(3)
		
		#get the speeds from shuffleboard
		'''intakeSpeed = wpilib.SmartDashboard.getNumber("Intake speed",self.intakeSpeed)
		halfMoonSpeed = wpilib.SmartDashboard.getNumber("Half moon speed",self.halfMoonSpeed)
		feederSpeed = wpilib.SmartDashboard.getNumber("Feeder speed",self.feederSpeed)
		turretSpeed = wpilib.SmartDashboard.getNumber("Turret speed",self.turretSpeed)'''
		
		#flywheelSpeed = .5*(self.auxiliaryJoystick.getRawAxis(1)-1)
		flywheelSpeed = self.flywheelSpeed
		
		#manual controls
		#if autoManualSwitch:
		
		#intake control
		if intakeIn:
			self.intake.collect(self.intakeSpeed,self.halfMoonSpeed) #intake speed, half moon speed
		elif intakeOut:
			self.intake.expel(self.intakeSpeed,self.halfMoonSpeed) #intake speed, half moon speed
		else:
			self.intake.stop()
		#feeder control
		if feederIn and manualFlywheel:
			self.feeder.feed(self.feederSpeed) #feeder speed
		else:
			self.feeder.stop()
		#flywheel control
		if manualFlywheel:
			self.turret.flywheelManual(flywheelSpeed) #percent
		else:
			self.turret.flywheelManual(0)
		#turret rotation
		if turretClockwise > 0.8:
			self.turret.turretManual(-self.turretSpeed) #turret speed
		elif turretCounterclockwise > 0.8:
			self.turret.turretManual(self.turretSpeed) #turret speed
		else:
			self.turret.turretManual(0)
		#climb control
		if climbUp:
			self.climb.extend(.35)
		elif climbDown:
			self.climb.contract(.35)
		else:
			self.climb.stop()
		
	def autonomousInit(self):
		self.drive.zeroEncoders()
		self.climb.zeroEncoder()
		self.feeder.zeroEncoder()
		self.navx.reset()
		
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('autonomous started')
		
	def autonomousPeriodic(self):
		location = self.drive.frontLeftPosition()
		#I hate this even more
		feederClock = self.feeder.getPosition()
		
		self.turret.manualFlywheel(self.flywheelSpeed)
		
		if self.dumbCounter < 10:
			self.dumbCounter += 1
		elif feederClock < self.dumbFeederDistance:
			self.feeder.feed(self.feederSpeed)
		elif location > self.dumbAutoDistance:
			self.turret.flywheelManual(0)
			self.feeder.stop()
			difference = self.dumbAutoDistance - location
			speed = difference/(self.dumbAutoDistance*10) + .07
			self.drive.move(0,speed,0)
		else:
			self.drive.stationary()
		
	def teleopInit(self):
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('teleop started')
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		#scale = wpilib.SmartDashboard.getNumber("Joystick scale factor", self.scaling)
		
		x = self.scaling*self.checkDeadband(self.joystick.getX(),True)
		y = -self.scaling*self.checkDeadband(self.joystick.getY(),True)
		z = self.scaling*self.checkDeadband(self.joystick.getZ(),False)
		
		if self.joystick.getRawButton(12):
			x *= .25
			y *= .25
			z *= .25
		
		angle = -1*self.navx.getAngle() + 90
		#wpilib.SmartDashboard.putNumber("angle",angle)
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
		self.feeder.zeroEncoder()
		
		print("Starting tests")
		
	def testPeriodic(self):
		'''self.checkSwitches()
		position = self.feeder.getPosition()
		wpilib.SmartDashboard.putNumber("feed encoder",position)
		drivePosition = self.drive.frontLeftPosition()
		wpilib.SmartDashboard.putNumber("front left drive",drivePosition)'''
		self.drive.checkEncoders()
		
	def disabledInit(self):
		self.drive.coast()
		self.turret.coast()
		self.feeder.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)