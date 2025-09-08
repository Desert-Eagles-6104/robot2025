// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemConfiguration;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemFactory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;



public class Intake extends SubsystemBase {
  private TalonFX masterFx; 
  private PhoenixPIDController phoenixPIDController;
  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0).withSlot(1);
  MotorConstants m_armMotorConstants;
  MotorConstants m_Configuration;

  public Intake(MotorConstants configuration) {
    masterFx = MotorSubsystemFactory.createTalonFX(configuration);
  }

  public double toRotations(double units) {
    return units * m_Configuration.rotationsPerPositionUnit;
  }

  public void driveToPoint(double position){
    armMotor.setControl(m_PositionVoltageRequest.withPosition(toRotations(position)).withSlot(1));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
