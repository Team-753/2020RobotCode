#!/usr/bin/env python3

import wpilib
import math
import rev
from wpilib import controller as controller
from DriveTrain import DriveTrain
from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from Turret import Turret
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.navx = AHRS.create_spi()
		self.navx.reset()
		
		self.joystick = wpilib.Joystick(0)
		self.auxiliaryJoystick = wpilib.Joystick(1)
		
		
		self.drive = DriveTrain()
		self.climb = Climb(12) #climb ID
		self.feeder = Feeder(9) #feeder ID
		self.intake = Intake(11,10) #intake ID, half moon ID
		self.turret = Turret(15,14) #rotate ID, flywheel ID
		
		self.drive.zeroEncoders()
		self.climb.coast()
		self.feeder.brake()
		self.intake.coast()
		self.turret.brake()
		
		#constants
		self.joystickDeadband = .2
		self.fieldOriented = True
		self.scaling = .8
		self.intakeSpeed = .35
		self.halfMoonSpeed = .8
		self.feederSpeed = .8
		self.flywheelRPM = 0
		self.turretSpeed = .1
		self.dumbAutoDistance = 20 #inches? ticks? meters? About 8 per foot, so...
		
		#put constants to shuffleboard
		wpilib.SmartDashboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putBoolean("Field Oriented", self.fieldOriented)
		wpilib.SmartDashboard.putNumber("Intake speed",self.intakeSpeed)
		wpilib.SmartDashboard.putNumber("Half moon speed",self.halfMoonSpeed)
		wpilib.SmartDashboard.putNumber("Feeder speed",self.feederSpeed)
		wpilib.SmartDashboard.putNumber("Turret speed",self.turretSpeed)
		
	def checkDeadband(self, axis):
		deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", self.joystickDeadband)
		if abs(axis) < deadband:
			axis = 0
		return axis
		
	def checkSwitches(self):
		#buttons list
		autoManualSwitch = self.auxiliaryJoystick.getRawButton(
		
		intakeIn = self.joystick.getRawButton(2)
		intakeOut = self.auxiliaryJoystick.getRawButton(
		
		feederIn = self.auxiliaryJoystick.getRawButton(
		feedOut = self.auxiliaryJoystick.getRawButton(
		
		manualFlywheel = self.auxiliaryJoystick.getRawButton(
		turretClockwise = self.auxiliaryJoystick.getRawButton(
		turretCounterclockwise = self.auxiliaryJoystick.getRawButton(
		
		climbUp = self.auxiliaryJoystick.getRawButton(
		climbDown = self.auxiliaryJoystick.getRawButton(
		
		#get the speeds from shuffleboard
		intakeSpeed = wpilib.SmartDashboard.getNumber("Intake speed",self.intakeSpeed)
		halfMoonSpeed = wpilib.SmartDashboard.getNumber("Half moon speed",self.halfMoonSpeed)
		feederSpeed = wpilib.SmartDashboard.getNumber("Feeder speed",self.feederSpeed)
		flywheelRPM = wpilib.SmartDashboard.getNumber("Flywheel RPM",self.flywheelRPM)
		turretSpeed = wpilib.SmartDashboard.getNumber("Turret speed",self.turretSpeed)
		flywheelSpeed = self.auxiliaryJoystick.getRawButton(
		
		#manual controls
		#if autoManualSwitch:
		
		#intake control
		if intakeIn:
			self.intake.collect(intakeSpeed,halfMoonSpeed) #intake speed, half moon speed
		elif intakeOut:
			self.intake.expel(intakeSpeed,halfMoonSpeed) #intake speed, half moon speed
		else:
			self.intake.stop()
		#feeder control
		if feederIn:
			self.feeder.feed(feederSpeed) #feeder speed
		elif feederOut:
			self.feeder.reverse() #feeder speed
		else:
			self.feeder.stop()
		#flywheel control
		if manualFlywheel:
			self.turret.flywheelManual(flywheelRPM) #rpm
		else:
			self.turret.flywheelManual(0)
		#turret rotation
		if turretClockwise:
			self.turret.turretManual(-turretSpeed) #turret speed
		elif turretCounterclockwise:
			self.turret.turretManual(turretSpeed) #turret speed
		else:
			self.turret.turretManual(0)
		
	def autonomousInit(self):
		self.drive.zeroEncoders()
		self.navx.reset()
		print('autonomous started')
		
	def autonomousPeriodic(self):
		location = self.drive.drivePositions()
		#shooting stuff from teleop
		if location < self.dumbAutoDistance:
			difference = self.dumbAutoDistance - location
			speed = difference/(self.dumbAutoDistance*10) + .1
			self.drive.move(0,speed,0)
			self.intake.collect(.35,.8)
		else:
			self.drive.stationary()
		#shooting stuff from teleop
		
	def teleopInit(self):
		self.drive.brake()
		
		print('teleop started')
		self.intake.stop()
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		scale = wpilib.SmartDashboard.getNumber("Joystick scale factor", self.scaling)
		fieldOriented = wpilib.SmartDashboard.getBoolean("Field Oriented", self.fieldOriented)
		
		x = scale*self.checkDeadband(self.joystick.getX())
		y = -scale*self.checkDeadband(self.joystick.getY())
		z = scale*self.checkDeadband(self.joystick.getZ())
		
		angle = -1*self.navx.getAngle() + 90
		wpilib.SmartDashboard.putNumber("angle",angle)
		angle *= math.pi/180
		
		'''if fieldOriented:
			cos = math.cos(angle)
			sin = math.sin(angle)
			temp = x*sin - y*cos
			y = x*cos + y*sin
			x = temp'''
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
			
		print(self.drive.drivePositions())
	def testInit(self):
		self.drive.coast()
		print("Starting tests")
		
	def testPeriodic(self):
		self.DriveTrain.checkEncoders()
		
	def disabledInit(self):
		self.drive.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)