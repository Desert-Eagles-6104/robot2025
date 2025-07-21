package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemManualControl extends Command {
  private MotorSubsystemTalon motorSubsystemTalon;
  private DoubleSupplier precentOutput;

	public MotorSubsystemManualControl(MotorSubsystemTalon motorSubsystemTalon, DoubleSupplier joystickPower) {
		this.motorSubsystemTalon = motorSubsystemTalon;
		this.precentOutput = joystickPower;
		addRequirements(motorSubsystemTalon);
	}
  
	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.motorSubsystemTalon.setPrecentOutput(this.precentOutput.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		this.motorSubsystemTalon.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
