// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class IntakeCoralSubsystem extends SubsystemBase {
  private static IntakeCoralSubsystem m_instance = null;

   private TalonFX m_motor;
   private BeamBreak m_beamBreak;
   private boolean m_isBroken;

   private TalonFXConfiguration configuration;

   private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private StatusSignal<Angle> m_positionSignal;
  private StatusSignal<AngularVelocity> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;
  

  public IntakeCoralSubsystem() {
    configuration = new TalonFXConfiguration();
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(Constants.IntakeCoralSubsystem.motorInverted).withDutyCycleNeutralDeadband(Constants.IntakeCoralSubsystem.DutyCycleNeutralDeadband));
    configuration.withSlot0(new Slot0Configs().withKS(Constants.IntakeCoralSubsystem.Ks).withKV(Constants.IntakeCoralSubsystem.Kv).withKA(Constants.IntakeCoralSubsystem.Ka).withKP(Constants.IntakeCoralSubsystem.Kp).withKI(Constants.IntakeCoralSubsystem.Ki).withKD(Constants.IntakeCoralSubsystem.Kd));
    configuration.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.IntakeCoralSubsystem.SensorToMechanismRatio));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.IntakeCoralSubsystem.supplyCurrentLimit).withSupplyCurrentLimitEnable(Constants.IntakeCoralSubsystem.SupplyCurrentLimitEnable));

    m_motor = new TalonFX(Constants.IntakeCoralSubsystem.motorId);
    m_motor.getConfigurator().apply(configuration);

    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.IntakeCoralSubsystem.frequencyHz ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);

    m_beamBreak = new BeamBreak(Constants.IntakeCoralSubsystem.beamBreakPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void disableMotors(){
    m_motor.disable();
  }

  public double getPosition(){
    return (m_positionSignal.getValueAsDouble());
  }

  public double getVelocity(){
    return m_velocitySignal.getValueAsDouble();
  }

  public boolean getBeamBreak(){
    return m_isBroken;
  }

  public void setMotorPrecent(double prcent){
    m_motor.set(prcent);
  }

  public void setPosition(double position){
    m_motor.setControl(m_PositionVoltageRequest.withPosition((getPosition() + position)));
  }


  public static IntakeCoralSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new IntakeCoralSubsystem();
    }  
    return m_instance;
  }
}
