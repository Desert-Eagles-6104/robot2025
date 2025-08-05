package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class MotorSubsystemTalon extends SubsystemBase {

	/** Creates a new ServoSubsystem. */
	public MotorSubsystemConfiguration configuration;

	private TalonFX masterFx; // creation of the master motor controller this is the controller we give all the command to.
	private TalonFX[] slaveFX; // creation of the slave controller this motor follows everything the master does.

	private double setpoint;

	// Requests
	private MotionMagicVoltage motiongMagicVoltageRequest = new MotionMagicVoltage(0).withSlot(0);
	private PositionVoltage positionVoltageRequest = new PositionVoltage(0).withSlot(1);
	private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
	private final VoltageOut sysidControlRequest = new VoltageOut(0);
	private MotionMagicVelocityVoltage motiongMagicVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
	private VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

	// Signals
	private final StatusSignal<Angle> positionSignal;
	private final StatusSignal<AngularVelocity> velocitySignal;
	private final StatusSignal<AngularAcceleration> accelerationSigna;
	private final StatusSignal<Current> supplyCurrentSignal;
	private final StatusSignal<Current> statorCurrentSignal;
	private final StatusSignal<Double> closedLoopError;
	private final StatusSignal<Voltage> appliedVoltageSignal;

	/**
	 * creation of the servoSubsytem constructor to define the objects and
	 * constants in the subsystem
	 * 
	 * @param configuration the configuration holds all of the values of the
	 * subsystem.
	 */
	public MotorSubsystemTalon(MotorSubsystemConfiguration configuration) {
		this.configuration = configuration;
		this.masterFx = MotorSubsystemFactory.createTalonFX(configuration.master);
		if (configuration.slaves != null) {
			this.slaveFX = MotorSubsystemFactory.createSlaveTalonsFX(configuration);
		}

		// Init signals
		this.positionSignal = this.masterFx.getPosition();
		this.velocitySignal = this.masterFx.getVelocity();
		this.accelerationSigna = this.masterFx.getAcceleration();
		this.supplyCurrentSignal = this.masterFx.getSupplyCurrent();
		this.statorCurrentSignal = this.masterFx.getStatorCurrent();
		this.appliedVoltageSignal = this.masterFx.getMotorVoltage();
		this.closedLoopError = this.masterFx.getClosedLoopError();

		BaseStatusSignal.setUpdateFrequencyForAll(
			50,
			this.closedLoopError,
			this.positionSignal,
			this.velocitySignal,
			this.accelerationSigna,
			this.appliedVoltageSignal,				 
			this.supplyCurrentSignal,
			this.statorCurrentSignal
		);

		this.masterFx.optimizeBusUtilization();
		this.resetSubsystemToInitialState();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		BaseStatusSignal.refreshAll(
			this.closedLoopError,
			this.positionSignal,
			this.velocitySignal,
			this.accelerationSigna,
			this.supplyCurrentSignal,
			this.statorCurrentSignal,
			this.appliedVoltageSignal
		);

		SmartDashboard.putNumber(this.configuration.subsystemName + " Position", this.getPosition());
		SmartDashboard.putNumber(this.configuration.subsystemName + " Velocity", this.getVelocity());
		SmartDashboard.putNumber(this.configuration.subsystemName + " closedLoopError", this.getClosedLoopError());
		SmartDashboard.putNumber(this.configuration.subsystemName + "current", this.getMotorCurrent());
		SmartDashboard.putBoolean(this.configuration.subsystemName + " AtSetpint", this.isAtSetpoint());
	}

	public void resetSubsystemToInitialState() {
		this.resetPosition(this.configuration.homePosition);
	}

	public void disableMotors() {
		this.masterFx.disable();
	}

	public void runCharacterization(Voltage volts) {
		this.masterFx.setControl(this.sysidControlRequest.withOutput(volts.in(Volts)));
	}

	public double toRotations(double units) {
		return units * this.configuration.master.rotationsPerPositionUnit;
	}

	public double fromRotations(double rotations) {
		return rotations / this.configuration.master.rotationsPerPositionUnit;
	}

	public void setMotionMagicPosition(double position) {
		this.setpoint = position;
		this.masterFx.setControl(this.motiongMagicVoltageRequest.withPosition(toRotations(position)).withSlot(0));
	}

	public void setPosition(double position) {
		this.setpoint = position;
		this.masterFx.setControl(this.positionVoltageRequest.withPosition(toRotations(position)).withSlot(1));
	}

	public void ControlSoftLimit(boolean enableSoftLimit) {
		this.masterFx.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitEnable(enableSoftLimit) // The initial value given to the configuration.
			.withForwardSoftLimitThreshold(this.configuration.master.getTalonFXConfig().SoftwareLimitSwitch.ForwardSoftLimitThreshold)
		);
	}

	public void setPrecentOutput(double precent) {
		this.masterFx.setControl(this.dutyCycleRequest.withOutput(precent));
	}

	public double getPosition() {
		return this.fromRotations(this.positionSignal.getValueAsDouble());
	}

	public void resetPosition(double position) {
		this.masterFx.setPosition(this.toRotations(position));
		if (this.slaveFX != null) {
			for (TalonFX talonFX : this.slaveFX) {
				talonFX.setPosition(position);
			}
		}
	}

	public boolean isAtSetpoint() {
		return Math.abs(this.closedLoopError.getValue()) < this.configuration.allowableError;
	}

	public double getVelocity() {
		return this.velocitySignal.getValueAsDouble();
	}

	public double getMotorCurrent() {
		return this.masterFx.getSupplyCurrent().getValueAsDouble();
	}

	public double getClosedLoopError() {
		return this.setpoint - this.getPosition();
	}

	public void changeNeutralMode(NeutralModeValue NeutralMode) {
		this.masterFx.setNeutralMode(NeutralMode);
		if (this.slaveFX != null) {
			for (TalonFX talonFX : this.slaveFX) {
				talonFX.setNeutralMode(NeutralMode);
			}
		}
	}

	public void setMotionMagicVelocity(double velocity) {
		this.masterFx.setControl(this.motiongMagicVelocityRequest.withVelocity(this.toRotations(velocity)));
	}

	public void setVelocity(double velocity) {
		this.masterFx.setControl(this.velocityVoltageRequest.withVelocity(this.toRotations(velocity)));
	}

}