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
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem m_instance = null;

  private TalonFX m_motor;
 
  
  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private TalonFXConfiguration configuration;

  private StatusSignal<Angle> m_positionSignal;
  private StatusSignal<AngularVelocity> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;

  public IntakeSubsystem() {
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(Constants.Intake.motorInverted).withDutyCycleNeutralDeadband(Constants.Intake.DutyCycleNeutralDeadband));
    configuration.withSlot0(new Slot0Configs().withKS(Constants.Intake.Ks).withKV(Constants.Intake.Kv).withKA(Constants.Intake.Ka).withKP(Constants.Intake.Kp).withKI(Constants.Intake.Ki).withKD(Constants.Intake.Kd));
    configuration.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Intake.SensorToMechanismRatio));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Intake.supplyCurrentLimit).withSupplyCurrentLimitEnable(Constants.Intake.SupplyCurrentLimitEnable));

    m_motor = new TalonFX(Constants.Intake.motorId);
    m_motor.getConfigurator().apply(configuration);

    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Intake.frequencyHz ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void disableMotors(){
    m_motor.disable();
  }

  public void setMotorPercent(double percent){
    m_motor.set(percent);
  }


  public static IntakeSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new IntakeSubsystem();
    }  
    return m_instance;
  }
}
