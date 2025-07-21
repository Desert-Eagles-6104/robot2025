// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Base.Motor;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.CSV.CSVReader;
import frc.DELib25.Intepulation.LinearInterpolator;
import frc.DELib25.Subsystems.MotorSubsystems.Base.IVelocitySubsystemBase;

public class VelocitySubsystemTalon extends SubsystemBase{
	/** Creates a new ServoSubsystem. */
	private VelocitySubsystemConfiguration m_configuration;

	private TalonFX m_masterFx;
	@SuppressWarnings("unused")
	private TalonFX[] m_slaveFX;

	// Requests
	private MotionMagicVelocityVoltage m_motiongMagicVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
	private VelocityVoltage m_VelocityVoltageRequest = new VelocityVoltage(0);
	private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
	private final VoltageOut m_sysidControlRequest = new VoltageOut(0);

	private final StatusSignal<Angle> m_positionSignal;
	private final StatusSignal<AngularVelocity> m_velocitySignal;
	private final StatusSignal<AngularAcceleration> m_accelerationSignal;
	private final StatusSignal<Current> m_supplyCurrentSignal;
	private final StatusSignal<Current> m_statorCurrentSignal;
	private final StatusSignal<Double> m_closedLoopError;
	private final StatusSignal<Voltage> m_appliedVoltageSignal;

	private double[][] m_shootingTable;
	private LinearInterpolator linearInterpolator;
	
	public VelocitySubsystemTalon(VelocitySubsystemConfiguration configuration) {
		m_configuration = configuration;
		m_masterFx = VelocitySubsystemMotorFactory.createTalonFX(configuration);
		if (configuration.slaves != null) {
			m_slaveFX = VelocitySubsystemMotorFactory.createSlaveTalonFX(configuration);
		}

		// Init signals
		m_positionSignal = m_masterFx.getPosition();
		m_velocitySignal = m_masterFx.getVelocity();
		m_accelerationSignal = m_masterFx.getAcceleration();
		m_supplyCurrentSignal = m_masterFx.getSupplyCurrent();
		m_statorCurrentSignal = m_masterFx.getStatorCurrent();
		m_appliedVoltageSignal = m_masterFx.getMotorVoltage();
		m_closedLoopError = m_masterFx.getClosedLoopError();
		BaseStatusSignal.setUpdateFrequencyForAll(50, m_closedLoopError, m_positionSignal, m_velocitySignal,
				m_accelerationSignal, m_appliedVoltageSignal, m_supplyCurrentSignal, m_statorCurrentSignal);
		m_masterFx.optimizeBusUtilization();

		setShootingTable(m_configuration.fileLocation);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		BaseStatusSignal.refreshAll(
				m_closedLoopError,
				m_positionSignal,
				m_velocitySignal,
				m_accelerationSignal,
				m_supplyCurrentSignal,
				m_statorCurrentSignal,
				m_appliedVoltageSignal);
		SmartDashboard.putNumber(m_configuration.subsystemName + " Velocity", getVelocity());
	}

	@Override
	public void disableMotors() {
		m_masterFx.disable();
	}

	@Override
	public void runCharacterization(Voltage volts) {
		m_masterFx.setControl(m_sysidControlRequest.withOutput(volts.in(Volts)));
	}

	@Override
	public double toRotations(double units) {
		return units * m_configuration.rotationsPerPositionUnit;
	}

	@Override
	public double fromRotations(double rotations) {
		return rotations / m_configuration.rotationsPerPositionUnit;
	}

	@Override
	public void setMotionMagicVelocity(double velocity) {
		m_masterFx.setControl(m_motiongMagicVelocityRequest.withVelocity(toRotations(velocity)));
	}

	@Override
	public void setVelocity(double velocity) {
		m_masterFx.setControl(m_VelocityVoltageRequest.withVelocity(toRotations(velocity)));
	}

	@Override
	public void setPrecentOutput(double precent) {
		m_masterFx.setControl(m_dutyCycleRequest.withOutput(precent));
	}

	@Override
	public boolean isAtSetpoint() {
		return Math.abs(m_closedLoopError.getValue()) < m_configuration.allowableError;
	}

	@Override
	public double getVelocity() {
		return m_velocitySignal.getValueAsDouble();
	}

	@Override
	public void log(BooleanSupplier staetLog) {
		// only for spark
	}

	@Override
	public void setShootingTable(String Filelocation) {
		try {
			CSVReader reader = new CSVReader(Filelocation);
			m_shootingTable = reader.readAsDouble(2);
			linearInterpolator = new LinearInterpolator(m_shootingTable);
		} catch (IOException e) {

		}
	}

	@Override
	public double getMotorCurrent() {
		return m_masterFx.getSupplyCurrent().getValueAsDouble();
	}

	@Override
	public void setUsingInterpulation(double value) {
		double speed = linearInterpolator.getInterpolatedValue(value);
		setVelocity(speed);
	}

	@Override
	public void setUsingInterpulationMotionMagic(double value) {
		double speed = linearInterpolator.getInterpolatedValue(value);
		setMotionMagicVelocity(speed);
	}

	@Override
	public double getInterpulationVelocity(double value) {
		return linearInterpolator.getInterpolatedValue(value);
	}
}