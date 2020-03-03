import wpilib
import rev

class Climb:
	encoderConversionFactor = 64 #this isn't real and needs to be changed
	def __init__(self, motorID):
		self.climbMotor = rev.CANSparkMax(motorID, rev.MotorType.kBrushless)
		self.climbEncoder = self.climbMotor.getEncoder()
		self.climbEncoder.setPositionConversionFactor(self.encoderConversionFactor)
		
	def extend(self,speed):
		self.climbMotor.set(-speed)
		
	def contract(self,speed):
		self.climbMotor.set(speed)
		
	def coast(self):
		self.climbMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def stop(self):
		self.climbMotor.set(0)
		
	def brake(self): #this is a forbidden method please never use it
		self.climbMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def checkEncoder(self):
		position = self.climbEncoder.getPosition()
		return position
		
	def zeroEncoder(self):
		self.climbEncoder.setPosition(0)
		