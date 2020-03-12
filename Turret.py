import wpilib
import rev
import math
from wpilib import controller as controller

class Turret:
	def __init__(self, turretID, flywheelID):
		self.turretMotor = rev.CANSparkMax(turretID, rev.MotorType.kBrushless)
		self.flywheelMotor = rev.CANSparkMax(flywheelID, rev.MotorType.kBrushless)
		
		self.turretEncoder = self.turretMotor.getEncoder()
		self.flywheelEncoder = self.flywheelMotor.getEncoder()
		
		self.flywheelMotor.setInverted(True)
		self.flywheelMotor.enableVoltageCompensation(12)
		
		self.turretEncoder.setPositionConversionFactor(2)
		
		self.kPTurretFar = .019
		self.kITurretFar = 0
		self.kDTurretFar = 0
		
		self.kPTurretNear = .006
		self.kITurretNear = .006
		self.kDTurretNear = 0
		
		turretMinOutput = -1
		turretMaxOutput = 1
		
		kPFlywheel = .0001
		kIFlywheel = 0
		kDFlywheel = 0
		self.kSFlywheel = .101
		kFFFlywheel = .0001751
		flywheelMinOutput = -1
		flywheelMaxOutput = 1
		
		self.turretMinPosition = -80
		self.turretMaxPosition = 80
		
		self.turretToleranceFar = 3 #degrees
		self.turretToleranceNear = .1 #degrees
		self.flywheelTolerance = 35 #rpm
		
		self.turretControllerFar = wpilib.controller.PIDController(self.kPTurretFar,self.kITurretFar,self.kDTurretFar)
		self.turretControllerFar.setSetpoint(0)
		self.turretControllerFar.setTolerance(self.turretToleranceFar,0)
		self.turretControllerFar.setIntegratorRange(-50,50) #idk what this does
		
		self.turretControllerNear = wpilib.controller.PIDController(self.kPTurretNear,self.kITurretNear,self.kDTurretNear)
		self.turretControllerNear.setSetpoint(0)
		self.turretControllerNear.setTolerance(self.turretToleranceNear,0)
		self.turretControllerNear.setIntegratorRange(-50,50)
		
		self.flywheelController = self.flywheelMotor.getPIDController()
		self.flywheelController.setP(kPFlywheel)
		self.flywheelController.setI(kIFlywheel)
		self.flywheelController.setD(kDFlywheel)
		self.flywheelController.setFF(kFFFlywheel)
		self.flywheelController.setOutputRange(flywheelMinOutput,flywheelMaxOutput)
		
		self.dNaught = 221 #icnhes

		self.aNaught = 5946
		self.hNaught = 110 #determined
		
	def calculateSpeed(self,distance):
		velocity = wpilib.SmartDashboard.getNumber("Shooter RPM",0) #this is an accurate model I promise
		return(velocity)
		
	def aim(self,height,yaw):
		try:
			distance = (self.hNaught/height)*self.dNaught
			
		except:
			print("no length :(")
			distance = 1
		velocity = self.calculateSpeed(distance)
		wpilib.SmartDashboard.putNumber("distance",distance)
		self.turretAlign(yaw,velocity)
		
	def flywheelManual(self,speed):
		self.flywheelMotor.setVoltage(speed)
		
	def flywheelRPM(self,rpm):
		self.flywheelController.setReference(rpm,rev.ControlType.kVelocity,0,self.kSFlywheel)
		
	def turretManual(self,speed):
		self.turretMotor.set(speed)
		
	def getFlywheelSpeed(self):
		speed = self.flywheelEncoder.getVelocity()
		return(speed)
		
	def turretAlign(self,yaw,velocity):
		position = self.turretEncoder.getPosition()
		
		if self.turretMinPosition < position < self.turretMaxPosition:
			if self.turretControllerFar.atSetpoint():
				if self.turretControllerNear.atSetpoint():
					turretOutput = -self.turretControllerNear.calculate(yaw)
				else:
					turretOutput = 0
			else:
				turretOutput = -self.turretControllerFar.calculate(yaw)
		else:
			turretOutput = 0
		
		self.turretMotor.set(turretOutput)
		self.flywheelRPM(velocity)
		velocityDifference = velocity - self.flywheelEncoder.getVelocity()
		if self.turretControllerNear.atSetpoint() and (abs(velocityDifference) < self.flywheelTolerance):
			wpilib.SmartDashboard.putBoolean("Ready to fire!",True)
		else:
			wpilib.SmartDashboard.putBoolean("Ready to fire!",False)
		
	def coast(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kCoast)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kBrake)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kBrake)
		