import wpilib
import rev

class Intake:
	def __init__(self, intakeID, halfMoonID):
		self.intakeMotor = rev.CANSparkMax(intakeID, rev.MotorType.kBrushless)
		self.halfMoonMotor = rev.CANSparkMax(halfMoonID, rev.MotorType.kBrushless)
		
	def collect(self,intakeSpeed,halfMoonSpeed):
		self.intakeMotor.set(-intakeSpeed)
		self.halfMoonMotor.set(-halfMoonSpeed)
		
	def expel(self,intakeSpeed,halfMoonSpeed):
		self.intakeMotor.set(intakeSpeed)
		self.halfMoonMotor.set(halfMoonSpeed)
		
	def stop(self):
		self.intakeMotor.set(0)
		self.halfMoonMotor.set(0)
		
	def coast(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def brake(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kBrake)
		