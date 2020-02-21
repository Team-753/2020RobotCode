import wpilib
import rev

class Turret: #this is currently just manual control, but motors can only be instantiated once so
	def __init__(self, rotateID, flywheelID): #eventually all the turret stuff will go here
		self.rotateMotor = rev.CANSparkMax(rotateID, rev.MotorType.kBrushless)
		self.flywheelMotor = rev.CANSparkMax(flywheelID, rev.MotorType.kBrushless)
		self.rotateEncoder = self.rotateMotor.getEncoder()
		self.flywheelEncoder = self.flywheelMotor.getEncoder()
		
		kPTurret = .002
		kITurret = .001
		kDTurret = 0
		
		self.turretController = wpilib.controller.PIDController(kPTurret,kITurret,kDTurret)
		
	def flywheelManual(self,rpm):
		speed = rpm/5700
		self.flywheelMotor.set(-speed)
		
	def turretManual(self,speed):
		self.rotateMotor.set(speed)
		
	def rotate(self):
		if self.rotateEncoder < threshold:
			pass
		
	def coast(self):
		self.rotateMotor.setIdleMode(rev.IdleMode.kCoast)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.rotateMotor.setIdleMode(rev.IdleMode.kBrake)
		self.flywheelMotor.setIdleMode(rev.IdleMode.kBrake)
		