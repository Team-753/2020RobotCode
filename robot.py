#!/usr/bin/env python3

import wpilib
import math
import rev
from wpilib import controller as controller
from DriveTrain import DriveTrain
#from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from Turret import Turret
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.navx = AHRS.create_spi()
		self.navx.reset()
		
		self.joystick = wpilib.Joystick(0)
		self.auxiliaryJoystick1 = wpilib.Joystick(1)
		self.auxiliaryJoystick2 = wpilib.Joystick(2)
		
		
		self.drive = DriveTrain()
		#self.climb = Climb(12) #climb ID
		self.feeder = Feeder(9) #feeder ID
		self.intake = Intake(11,10) #intake ID, half moon ID
		self.turret = Turret(15,14) #rotate ID, flywheel ID
		
		self.drive.zeroEncoders()
		#self.climb.coast()
		self.feeder.brake()
		self.intake.coast()
		self.turret.brake()
		
		#constants
		self.joystickDeadband = .2
		self.fieldOriented = False
		self.scaling = .5
		self.intakeSpeed = .35
		self.halfMoonSpeed = .75
		self.feederSpeed = .75
		self.flywheelRPM = 0
		self.turretSpeed = .1
		
		#put constants to shuffleboard
		wpilib.Shuffleboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putBoolean("Field Oriented", self.fieldOriented)
		wpilib.SmartDashboard.putNumber("Intake speed",self.intakeSpeed)
		wpilib.SmartDashboard.putNumber("Half moon speed",self.halfMoonSpeed)
		wpilib.SmartDashboard.putNumber("Feeder speed",self.feederSpeed)
		wpilib.SmartDashboard.putNumber("Flywheel RPM",self.flywheelRPM)
		wpilib.SmartDashboard.putNumber("Turret speed",self.turretSpeed)
		
	def checkDeadband(self, axis):
		deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", self.joystickDeadband)
		if abs(axis) < deadband:
			axis = 0
		return axis
		
	def checkSwitches(self):
		#buttons list
		autoManualSwitch = self.auxiliaryJoystick2.getRawButton(1)
		intakeIn = self.joystick.getRawButton(1)
		intakeOut = self.joystick.getRawButton(2)
		feederIn = self.auxiliaryJoystick1.getRawButton(1)
		feedOut = self.auxiliaryJoystick1.getRawButton(11)
		manualFlywheel = self.auxiliaryJoystick2.getRawButton(4)
		turretClockwise = self.auxiliaryJoystick1.getRawButton(3)
		turretCounterclockwise = self.auxiliaryJoystick1.getRawButton(4)
		
		#get the speeds from shuffleboard
		intakeSpeed = wpilib.SmartDashboard.getNumber("Intake speed",self.intakeSpeed)
		halfMoonSpeed = wpilib.SmartDashboard.getNumber("Half moon speed",self.halfMoonSpeed)
		feederSpeed = wpilib.SmartDashboard.getNumber("Feeder speed",self.feederSpeed)
		flywheelRPM = wpilib.SmartDashboard.getNumber("Flywheel RPM",self.flywheelRPM)
		turretSpeed = wpilib.SmartDashboard.getNumber("Turret speed",self.turretSpeed)
		
		#manual controls
		if autoManualSwitch:
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
			else:
				self.feeder.stop()
			'''elif feederOut:
				self.feeder.reverse() #feeder speed
			else:
				self.feeder.stop()'''
			#flywheel control
			if manualFlywheel:
				self.turret.flywheelManual(flywheelRPM) #rpm
			else:
				self.turret.flywheelManual(0)
			#turret rotation
			if turretClockwise:
				self.turret.turretManual(turretSpeed) #turret speed
			elif turretCounterclockwise:
				self.turret.turretManual(-turretSpeed) #turret speed
			else:
				self.turret.turretManual(0)
		
	def autonomousInit(self):
		self.drive.coast()
		print('autonomous started')
		
	def autonomousPeriodic(self):
		self.drive.checkEncoders()
		
	def teleopInit(self):
		self.drive.zeroEncoders()
		self.drive.brake()
		self.navx.reset() #please delete this before competition
		print('teleop started')
		
	def teleopPeriodic(self):
		self.checkSwitches()
		
		scale = wpilib.Shuffleboard.getNumber("Joystick scale factor", self.scaling)
		fieldOriented = wpilib.SmartDashboard.getBoolean("Field Oriented", self.fieldOriented)
		
		x = scale*self.checkDeadband(self.joystick.getX())
		y = -scale*self.checkDeadband(self.joystick.getY())
		z = scale*self.checkDeadband(self.joystick.getZ())
		
		angle = -1*self.navx.getAngle() + 90
		wpilib.SmartDashboard.putNumber("angle",angle)
		angle *= math.pi/180
		
		if fieldOriented:
			cos = math.cos(angle)
			sin = math.sin(angle)
			temp = x*sin - y*cos
			y = x*cos + y*sin
			x = temp
		
		if max(abs(x),abs(y),abs(z)) != 0:
			self.drive.move(x,y,z)
		else:
			self.drive.stationary()
		
	def disabledInit(self):
		self.drive.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)