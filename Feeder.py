import wpilib
import rev

class Feeder:
	def __init__(self, motorID):
		self.feederMotor = rev.CANSparkMax(motorID, rev.MotorType.kBrushless)
		self.servo = wpilib.PWM(0)
		
	def feed(self, speed):
		self.feederMotor.set(speed)
		self.servo.setPosition(.5)
		
	def reverse(self, speed): #this is a forbidden method but may be necessary
		self.servo.setPosition(.5)
		self.feederMotor.set(-speed)
		
	def stop(self):
		self.feederMotor.set(0)
		self.servo.setPosition(1)
		
	def coast(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kBrake)
		