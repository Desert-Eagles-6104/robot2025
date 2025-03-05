// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.ServoSubsystem.Base.Motor;

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
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.IServoSubsystemBase;

public class ServoSubsystemTalon extends SubsystemBase implements IServoSubsystemBase{
  /** Creates a new ServoSubsystem. */
  public ServoSubsystemConfiguration m_configuration;
  
  private TalonFX m_masterFx;
  private TalonFX[] m_slaveFX;
  public double setpoint;

    // Requests
  private MotionMagicVoltage m_motiongMagicVoltageRequest = new MotionMagicVoltage(0).withSlot(0);
  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0).withSlot(1);
  private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut m_sysidControlRequest = new VoltageOut(0);

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<AngularAcceleration> m_accelerationSignal;
  private final StatusSignal<Current> m_supplyCurrentSignal;
  private final StatusSignal<Current> m_statorCurrentSignal;
  private final StatusSignal<Double> m_closedLoopError;
  private final StatusSignal<Voltage> m_appliedVoltageSignal;
 
  public ServoSubsystemTalon(ServoSubsystemConfiguration configuration) {
    m_configuration =  configuration;
    m_masterFx = ServoSubsystemMotorFactory.createTalonFX(configuration);
    if(configuration.slaves != null){
      m_slaveFX = ServoSubsystemMotorFactory.createSlaveTalonFX(configuration);
    }
    

    // Init signals
    m_positionSignal = m_masterFx.getPosition();
    m_velocitySignal = m_masterFx.getVelocity();
    m_accelerationSignal = m_masterFx.getAcceleration();
    m_supplyCurrentSignal = m_masterFx.getSupplyCurrent();
    m_statorCurrentSignal = m_masterFx.getStatorCurrent();
    m_appliedVoltageSignal = m_masterFx.getMotorVoltage();
    m_closedLoopError = m_masterFx.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(50,m_closedLoopError, m_positionSignal, m_velocitySignal, m_accelerationSignal, m_appliedVoltageSignal, m_supplyCurrentSignal, m_statorCurrentSignal);
    m_masterFx.optimizeBusUtilization();
    resetSubsystemToInitialState();
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
      SmartDashboard.putNumber(m_configuration.subsystemName + " Position", getPosition());
      SmartDashboard.putNumber(m_configuration.subsystemName + " Velocity", getVelocity());
      SmartDashboard.putNumber(m_configuration.subsystemName + " closedLoopError", getClosedLoopError());
      SmartDashboard.putNumber(m_configuration.subsystemName + "current", getMotorCurrent());
      SmartDashboard.putBoolean(m_configuration.subsystemName + " AtSetpint", isAtSetpoint());
  }

  @Override
  public void resetSubsystemToInitialState() {
    resetPosition(m_configuration.homePosition);
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
  public void setMotionMagicPosition(double position) {
    setpoint = position;
    m_masterFx.setControl(m_motiongMagicVoltageRequest.withPosition(toRotations(position)).withSlot(0));
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    m_masterFx.setControl(m_PositionVoltageRequest.withPosition(toRotations(position)).withSlot(1));
  }

  
  @Override
  public void ControlSoftLimit(boolean enableSoftLimit) {  
    m_masterFx.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(enableSoftLimit)
    .withForwardSoftLimitThreshold(m_configuration.forwardSoftLimit));
  }

  @Override
  public void setPrecentOutput(double precent) {
    m_masterFx.setControl(m_dutyCycleRequest.withOutput(precent));
  }

  @Override
  public double getPosition(){
    return fromRotations(m_positionSignal.getValueAsDouble()); 
  }

  @Override
  public void resetPosition(double position) {
    m_masterFx.setPosition(toRotations(position));
    if(m_slaveFX != null){
      for(TalonFX talonFX : m_slaveFX){
        talonFX.setPosition(position);
      }
    }
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(setpoint - getPosition()) < m_configuration.allowableError;
  }

  @Override
  public double getMotorCurrent() {
    return m_masterFx.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getClosedLoopError() {
    return setpoint - getPosition();
  }

  @Override
  public double getVelocity() {
    return fromRotations(m_masterFx.getVelocity().getValueAsDouble());
  }

  @Override
  public void changeNeutralMode(NeutralModeValue NeutralMode) {
    m_masterFx.setNeutralMode(NeutralMode);
    if(m_slaveFX != null){
      for(TalonFX talonFX : m_slaveFX){
        talonFX.setNeutralMode(NeutralMode);
      }
    }
  }
}