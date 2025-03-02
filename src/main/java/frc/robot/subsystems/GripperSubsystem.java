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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
  private static GripperSubsystem m_instance = null;

   private TalonFX m_motor;
   private BeamBreak m_beamBreak;
   private boolean m_isBroken;
   private boolean m_hasGamePiece;

   private TalonFXConfiguration configuration;

   private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private StatusSignal<Angle> m_positionSignal;
  private StatusSignal<AngularVelocity> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;
  

  public GripperSubsystem() {
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs()
    .withInverted(Constants.Gripper.motorInverted)
    .withDutyCycleNeutralDeadband(Constants.Gripper.DutyCycleNeutralDeadband));
    configuration.withSlot0(new Slot0Configs().withKS(Constants.Gripper.Ks).withKV(Constants.Gripper.Kv).withKA(Constants.Gripper.Ka).withKP(Constants.Gripper.Kp).withKI(Constants.Gripper.Ki).withKD(Constants.Gripper.Kd));
    configuration.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Constants.Gripper.SensorToMechanismRatio));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Gripper.supplyCurrentLimit).withSupplyCurrentLimitEnable(Constants.Gripper.SupplyCurrentLimitEnable));
    m_hasGamePiece = false;

    m_motor = new TalonFX(Constants.Gripper.motorId);
    m_motor.getConfigurator().apply(configuration);

    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.Gripper.frequencyHz ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);

    m_beamBreak = new BeamBreak(Constants.Gripper.beamBreakPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_beamBreak.update();
    m_hasGamePiece = m_beamBreak.get();
    SmartDashboard.putBoolean("BeamBreak", m_hasGamePiece);
  }
  
  public void disableMotors(){
    m_motor.disable();
  }
  
  public void HasGamePieceTrue(){
    m_hasGamePiece = true;
  }

  public void HasGamePieceFalse(){
    m_hasGamePiece = false;
  }


  public boolean HasGamePiece(){
    if(m_motor.getSupplyCurrent().getValueAsDouble()>15.0){
      m_hasGamePiece= true;
    }
    return m_hasGamePiece;
  }

  public void setMotorPercent(double percent){
    m_motor.set(percent);
  }

  public static GripperSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new GripperSubsystem();
    }  
    return m_instance;
  }
}
