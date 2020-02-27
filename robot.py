#!/usr/bin/env python3

import wpilib
import math
from DriveTrain import DriveTrain
from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from Turret import Turret
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.navx = AHRS.create_spi()
		
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
		
		#constants
		self.joystickDeadband = .2
		self.scaling = .8
		self.intakeSpeed = .35
		self.halfMoonSpeed = .8
		self.feederSpeed = .8
		self.turretSpeed = .05
		self.climberSpeed = .35
		self.dumbAutoDistance = 20 #inches? ticks? meters? About 8 per foot, so...
		
		#put constants to shuffleboard
		wpilib.SmartDashboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putNumber("Intake speed",self.intakeSpeed)
		wpilib.SmartDashboard.putNumber("Half moon speed",self.halfMoonSpeed)
		wpilib.SmartDashboard.putNumber("Feeder speed",self.feederSpeed)
		wpilib.SmartDashboard.putNumber("Turret speed",self.turretSpeed)
		
		self.navx.reset()
		
	def checkDeadband(self, axis):
		deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", self.joystickDeadband)
		if abs(axis) < deadband:
			axis = 0
		return axis
		
	def checkSwitches(self):
		#buttons list
		autoManualSwitch = self.auxiliaryJoystick.getRawButton(7)
		
		intakeIn = self.joystick.getRawButton(2)
		intakeOut = self.auxiliaryJoystick.getRawButton(1)
		
		feederIn = self.auxiliaryJoystick.getRawButton(8)
		feedOut = self.auxiliaryJoystick.getRawButton(9)
		
		manualFlywheel = self.auxiliaryJoystick.getRawButton(6)
		turretClockwise = self.joystick.getRawButton(11)
		turretCounterclockwise = self.joystick.getRawButton(12)
		
		climbUp = self.auxiliaryJoystick.getRawButton(2)
		climbDown = self.auxiliaryJoystick.getRawButton(3)
		
		#get the speeds from shuffleboard
		intakeSpeed = wpilib.SmartDashboard.getNumber("Intake speed",self.intakeSpeed)
		halfMoonSpeed = wpilib.SmartDashboard.getNumber("Half moon speed",self.halfMoonSpeed)
		feederSpeed = wpilib.SmartDashboard.getNumber("Feeder speed",self.feederSpeed)
		turretSpeed = wpilib.SmartDashboard.getNumber("Turret speed",self.turretSpeed)
		flywheelSpeed = .5*(self.auxiliaryJoystick.getZ()-1)
		
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
			self.feeder.reverse(feederSpeed) #feeder speed
			print("I really don't like this button")
		else:
			self.feeder.stop()
		#flywheel control
		if manualFlywheel:
			self.turret.flywheelManual(flywheelSpeed) #percent
		else:
			self.turret.flywheelManual(0)
		#turret rotation
		if turretClockwise:
			self.turret.turretManual(-turretSpeed) #turret speed
		elif turretCounterclockwise:
			self.turret.turretManual(turretSpeed) #turret speed
		else:
			self.turret.turretManual(0)
		#climb control
		if climbUp:
			self.climb.extend(self.climbSpeed)
		elif climbDown:
			self.climb.contract(self.climbSpeed)
		else:
			self.climb.stop()
		
	def autonomousInit(self):
		self.drive.zeroEncoders()
		self.climb.zeroEncoder()
		self.navx.reset()
		
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
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
			self.intake.stop()
		#shooting stuff from teleop
		
	def teleopInit(self):
		self.drive.brake()
		self.feeder.brake()
		self.turret.brake()
		print('teleop started')
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		scale = wpilib.SmartDashboard.getNumber("Joystick scale factor", self.scaling)
		
		x = scale*self.checkDeadband(self.joystick.getX())
		y = -scale*self.checkDeadband(self.joystick.getY())
		z = scale*self.checkDeadband(self.joystick.getZ())
		
		if self.joystick.getRawButton(7):
			x *= .25
			y *= .25
			z *= .25
		
		angle = -1*self.navx.getAngle() + 90
		wpilib.SmartDashboard.putNumber("angle",angle)
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
		self.DriveTrain.checkEncoders()
		
	def disabledInit(self):
		self.drive.coast()
		self.turret.coast()
		self.feeder.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)