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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private static CoralIntakeSubsystem m_instance = null;

   private TalonFX m_motor;

   private BeamBreak m_beamBreak;
   private boolean m_isBroken;
   private boolean m_approve;

   private TalonFXConfiguration configuration;

   private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private StatusSignal<Angle> m_positionSignal;
  private StatusSignal<AngularVelocity> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;
  

  public CoralIntakeSubsystem() {
    configuration = new TalonFXConfiguration();
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(Constants.CoralIntake.motorInverted).withDutyCycleNeutralDeadband(Constants.CoralIntake.DutyCycleNeutralDeadband));
    configuration.withSlot0(new Slot0Configs().withKS(Constants.CoralIntake.Ks).withKV(Constants.CoralIntake.Kv).withKA(Constants.CoralIntake.Ka).withKP(Constants.CoralIntake.Kp).withKI(Constants.CoralIntake.Ki).withKD(Constants.CoralIntake.Kd));
    configuration.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.CoralIntake.SensorToMechanismRatio));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.CoralIntake.supplyCurrentLimit).withSupplyCurrentLimitEnable(Constants.CoralIntake.SupplyCurrentLimitEnable));
    m_approve = false;

    m_motor = new TalonFX(Constants.CoralIntake.motorId);
    m_motor.getConfigurator().apply(configuration);

    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.CoralIntake.frequencyHz ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);

    m_beamBreak = new BeamBreak(Constants.CoralIntake.beamBreakPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getApprove(){
    return m_approve;
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


  public static CoralIntakeSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new CoralIntakeSubsystem();
    }  
    return m_instance;
  }
}
