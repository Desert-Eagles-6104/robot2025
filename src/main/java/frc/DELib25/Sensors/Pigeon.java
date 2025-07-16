package frc.DELib25.Sensors;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Pigeon {
	private final Pigeon2 gyro;
	// Configs
	private boolean inverted = false;
	private Rotation2d yawAdjustmentAngle = Rotation2d.fromDegrees(0);
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	public Pigeon(int port, String bus) {
		this.gyro = new Pigeon2(port, bus);
		BaseStatusSignal.setUpdateFrequencyForAll(50, this.gyro.getRoll(), this.gyro.getPitch(), this.gyro.getYaw());
		this.gyro.reset();
		this.gyro.getConfigurator().apply(new Pigeon2Configuration());
		this.gyro.optimizeBusUtilization();
	}
	
	public Pigeon(int port) {
		this(port, "Canivore");
	}
	
	public Rotation2d getYaw() {
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.unaryMinus());
		if (inverted) {
			return angle.unaryMinus();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.unaryMinus());
	}

	public Rotation2d getPitch() {
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.unaryMinus()).unaryMinus();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
		.rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle =
		getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle =
		getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {
		return Rotation2d.fromDegrees(
		BaseStatusSignal.getLatencyCompensatedValueAsDouble(getYawStatusSignal(), getRateStatusSignal()));
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(this.gyro.getPitch().getValueAsDouble());
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(this.gyro.getRoll().getValueAsDouble());
	}

	public StatusSignal<Angle> getYawStatusSignal() {
		return this.gyro.getYaw();
	}

	public StatusSignal<AngularVelocity> getRateStatusSignal() {
		return this.gyro.getAngularVelocityZDevice();
	}

	public StatusSignal<AngularVelocity> getRateStatusSignalWorld(){
		return this.gyro.getAngularVelocityZWorld();
	}
}