package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.CSV.CSVReader;
import frc.DELib25.Interpolation.LinearInterpolator;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemInterpolatedVelocity extends Command{
    protected final MotorSubsystemTalon subsystemTalon;
	protected final LinearInterpolator interpolator;
	protected final DoubleSupplier valueSupplier;

	public MotorSubsystemInterpolatedVelocity(MotorSubsystemTalon subsystemTalon, double[][] table, DoubleSupplier valueSupplier) {
		this.subsystemTalon = subsystemTalon;
		this.interpolator = new LinearInterpolator(table);
		this.valueSupplier = valueSupplier;
		addRequirements(subsystemTalon);
	}
	public MotorSubsystemInterpolatedVelocity(MotorSubsystemTalon subsystemTalon, String csvPath, DoubleSupplier inputSupplier) throws IOException {
		this(subsystemTalon, new CSVReader(csvPath).readAsDouble(2), inputSupplier);
	}

	@Override
	public void execute() {
		double input = this.valueSupplier.getAsDouble();
		double velocity = this.interpolator.getInterpolatedValue(input);
		this.subsystemTalon.setVelocity(velocity);
	}

	@Override
	public void end(boolean interrupted) {
		subsystemTalon.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
