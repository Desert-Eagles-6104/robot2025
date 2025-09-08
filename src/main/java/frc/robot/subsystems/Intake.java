// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.controls.PIDVoltage;



public class Intake extends SubsystemBase {
  private TalonFX armMotor;
  private PhoenixPIDController phoenixPIDController;
  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0).withSlot(1);
  private ServoSubsystemConfiguration m_configuration;
  private PIDVoltage pidRequest;
  public Intake(ServoSubsystemConfiguration configuration) {
    armMotor = new TalonFX(5);
    this.phoenixPIDController = new PhoenixPIDController(0.3, 0.0, 0.0);
    m_configuration =  configuration;
    pidRequest = new PIDVoltage(phoenixPIDController);

  }
  public double toRotations(double units) {
    return units * m_configuration.rotationsPerPositionUnit;
  }
  public void driveToPoint(double position){
    armMotor.setControl(m_PositionVoltageRequest.withPosition(toRotations(position)).withSlot(1));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
