package frc.DELib25.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.EdgeDetectorBoolean;

public class BeamBreakSubsystem extends SubsystemBase {

	private final DigitalInput digitalInput;
	private EdgeDetectorBoolean edgeDetector;

	public BeamBreakSubsystem(int channel) {
		this.digitalInput = new DigitalInput(channel);
		this.edgeDetector = new EdgeDetectorBoolean();
	}

	@Override 
	public void periodic() {
		this.edgeDetector.risingEdge(this.digitalInput.get());
	}

	public boolean get() {
		return this.edgeDetector.getLastValue();
	}
}