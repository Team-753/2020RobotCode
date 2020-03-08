import wpilib
import rev

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
		kITurret = .06
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
		
		self.turretTolerance = .25 #degrees
		
		self.turretController = self.turretMotor.getPIDController()
		self.turretController.setP(kPTurret)
		self.turretController.setI(kITurret)
		self.turretController.setD(kDTurret)
		self.turretController.setOutputRange(turretMinOutput,turretMaxOutput)
		
		self.flywheelController = self.flywheelMotor.getPIDController()
		self.flywheelController.setP(kPFlywheel)
		self.flywheelController.setI(kIFlywheel)
		self.flywheelController.setD(kDFlywheel)
		self.flywheelController.setFF(kFFFlywheel)
		self.flywheelController.setOutputRange(flywheelMinOutput,flywheelMaxOutput)
		
	def flywheelManual(self,speed):
		self.flywheelMotor.setVoltage(speed)
		
	def flywheelRPM(self,rpm):
		self.flywheelController.setReference(rpm,rev.ControlType.kVelocity,0,self.kSFlywheel)
		
	def turretManual(self,speed):
		self.turretMotor.set(speed)
		
	def getFlywheelSpeed(self):
		speed = self.flywheelEncoder.getVelocity()
		return(speed)
		
	def turretPosition(self,goal):
		position = self.turretEncoder.getPosition()
		if abs(position - goal) > self.turretTolerance:
			self.turretController.setReference(goal,rev.ControlType.kPosition)
		else:
			self.turretMotor.set(0)
		
	def coast(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kCoast)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.turretMotor.setIdleMode(rev.IdleMode.kBrake)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kBrake)
		