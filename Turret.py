import wpilib
import rev

class Turret: #this is currently just manual control, but motors can only be instantiated once so
	def __init__(self, turretID, flywheelID): #eventually all the turret stuff will go here
		self.turretMotor = rev.CANSparkMax(turretID, rev.MotorType.kBrushless)
		self.flywheelMotor = rev.CANSparkMax(flywheelID, rev.MotorType.kBrushless)
		self.turretEncoder = self.turretMotor.getEncoder()
		self.flywheelEncoder = self.flywheelMotor.getEncoder()
		
		self.turretEncoder.setPositionConversionFactor(12.414)
		
		kPTurret = .007
		kITurret = .06
		kDTurret = 0
		kIZoneTurret = 3 #degrees in which position error is integrated
		turretMaxOutput = .25
		turretMinOutput = .25
		
		self.turretTolerance = .3 #degrees
		
		self.turretController = self.turretMotor.getPIDController()
		self.turretController.setP(kPTurret)
		self.turretController.setI(kITurret)
		self.turretController.setD(kDTurret)
		self.turretController.setIZone(kIZoneTurret)
		self.turretController.setOutputRange(turretMinOutput,turretMaxOutput)
		
		#self.lights = wpilib.PWM(0)
		
	def flywheelManual(self,speed):
		speed *= -1
		self.flywheelMotor.set(speed)
		
	def turretManual(self,speed):
		self.turretMotor.set(speed)
		
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
		