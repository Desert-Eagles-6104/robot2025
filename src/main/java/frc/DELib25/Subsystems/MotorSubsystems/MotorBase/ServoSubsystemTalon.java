// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystemConfiguration;
/**
 * here we create a servoSubsytem for a talonFX motorController
 *  a servo is a motor that gets to a certain position within an input in range of the specific system. */

public class ServoSubsystemTalon extends SubsystemBase {
  /** Creates a new ServoSubsystem. */
  public ServoSubsystemConfiguration m_configuration;
  
  private TalonFX m_masterFx; // creation of the master motor controller this is the controller we give all the command to.
  private TalonFX[] m_slaveFX; // creation of the slave controller this motor follows everything the master does.
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
  
  /**
   *  creation of the servoSubsytem constructor to define the objects and constants in the subsystem
   *  @param configuration the configuration holds all of the values of the subsystem.
   */
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

  public void resetSubsystemToInitialState() {
    resetPosition(m_configuration.homePosition);
  }

  public void disableMotors() {
    m_masterFx.disable();
  }

  public void runCharacterization(Voltage volts) {
    m_masterFx.setControl(m_sysidControlRequest.withOutput(volts.in(Volts)));
  }

  public double toRotations(double units) {
    return units * m_configuration.rotationsPerPositionUnit;
  }

  public double fromRotations(double rotations) {
    return rotations / m_configuration.rotationsPerPositionUnit;  
  }

  public void setMotionMagicPosition(double position) {
    setpoint = position;
    m_masterFx.setControl(m_motiongMagicVoltageRequest.withPosition(toRotations(position)).withSlot(0));
  }

  public void setPosition(double position) {
    setpoint = position;
    m_masterFx.setControl(m_PositionVoltageRequest.withPosition(toRotations(position)).withSlot(1));
  }

  public void ControlSoftLimit(boolean enableSoftLimit) {  
    m_masterFx.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(enableSoftLimit)
    .withForwardSoftLimitThreshold(m_configuration.forwardSoftLimit));
  }

  public void setPrecentOutput(double precent) {
    m_masterFx.setControl(m_dutyCycleRequest.withOutput(precent));
  }

  public double getPosition(){
    return fromRotations(m_positionSignal.getValueAsDouble()); 
  }

  public void resetPosition(double position) {
    m_masterFx.setPosition(toRotations(position));
    if(m_slaveFX != null){
      for(TalonFX talonFX : m_slaveFX){
        talonFX.setPosition(position);
      }
    }
  }

  public boolean isAtSetpoint() {
    return Math.abs(setpoint - getPosition()) < m_configuration.allowableError;
  }

  public double getMotorCurrent() {
    return m_masterFx.getSupplyCurrent().getValueAsDouble();
  }

  public double getClosedLoopError() {
    return setpoint - getPosition();
  }

  public double getVelocity() {
    return fromRotations(m_masterFx.getVelocity().getValueAsDouble());
  }

  public void changeNeutralMode(NeutralModeValue NeutralMode) {
    m_masterFx.setNeutralMode(NeutralMode);
    if(m_slaveFX != null){
      for(TalonFX talonFX : m_slaveFX){
        talonFX.setNeutralMode(NeutralMode);
      }
    }
  }
}