import wpilib
import rev
import math

class Turret:
	def __init__(self, turretID, flywheelID):
		self.turretMotor = rev.CANSparkMax(turretID, rev.MotorType.kBrushless)
		self.flywheelMotor = rev.CANSparkMax(flywheelID, rev.MotorType.kBrushless)
		
		self.turretEncoder = self.turretMotor.getEncoder()
		self.flywheelEncoder = self.flywheelMotor.getEncoder()
		
		self.flywheelMotor.setInverted(True)
		self.flywheelMotor.enableVoltageCompensation(12)
		
		self.turretEncoder.setPositionConversionFactor(12.414) #this definitely needs to change
		
		kPTurret = .007
		kITurret = .004
		kDTurret = 0
		kIZoneTurret = 3 #degrees in which position error is integrated
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
		
		self.turretTolerance = .25 #degrees
		self.flywheelTolerance = 30 #rpm
		
		self.turretController = wpilib.controller.PIDController(kPTurret,kITurret,kDTurret)
		self.turretController.setOutputRange(turretMinOutput,turretMaxOutput)
		
		self.flywheelController = self.flywheelMotor.getPIDController()
		self.flywheelController.setP(kPFlywheel)
		self.flywheelController.setI(kIFlywheel)
		self.flywheelController.setD(kDFlywheel)
		self.flywheelController.setFF(kFFFlywheel)
		self.flywheelController.setOutputRange(flywheelMinOutput,flywheelMaxOutput)
		
		self.dNaught = 18.0416
		self.aNaught = 5946
		
	def calculateSpeed(self,distance):
		velocity = 1 #this is an accurate model I promise
		return(velocity)
		
	def aim(self,width,height):
		area = width*height
		try:
			distance = (self.aNaught/area)*self.dNaught
		except:
			print("no area :(")
			distance = 1
		velocity = self.calculateSpeed(distance)
		self.turretAlign(sd.getEntry('targetYaw').getDouble(0),velocity)
		
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
		turretOutput = -1*self.turretController.calculate(yaw)
		position = self.turretEncoder.getPosition()
		
		if self.turretMinPosition < position < self.turretMaxPosition:
			if abs(yaw) < self.turretTolerance:
				self.turretController.setReference(goal,rev.ControlType.kPosition)
		else:
			self.turretMotor.set(0)
		self.flywheelRPM(velocity)
		velocityDifference = velocity - self.flywheelEncoder.getVelocity()
		if (abs(yaw) < self.turretTolerance) and (abs(velocityDifference) < self.flywheelTolerance):
			wpilib.SmartDashboard.putBoolean("Ready to fire!",True)
		else:
			wpilib.SmartDashboard.putBoolean("Ready to fire!",False)
		
	def coast(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kCoast)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kBrake)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kBrake)
		