package frc.DELib25.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase{

	private final DigitalInput digitalInput;

	public BeamBreakSubsystem(int channel) {
		this.digitalInput = new DigitalInput(channel);
	}

	public boolean get() {
		return this.digitalInput.get();
	}
}