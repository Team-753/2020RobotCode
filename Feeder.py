import wpilib
import rev

class Feeder:
	def __init__(self, motorID):
		self.feederMotor = rev.CANSparkMax(motorID, rev.MotorType.kBrushless)
		
	def feed(self, speed):
		self.feederMotor.set(speed)
		
	def reverse(self, speed): #this is a forbidden method but may be necessary
		self.feederMotor.set(-speed)
		
	def stop(self):
		self.feederMotor.set(0)
		
	def coast(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kBrake)
		