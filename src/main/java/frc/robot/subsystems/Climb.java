// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  TalonFX rightTalonFX;
  TalonFX leftTalonFX;
  /** Creates a new Climb. */
  public Climb() {
    rightTalonFX = new TalonFX(8 , "Canivore");
    leftTalonFX = new TalonFX(7 , "Canivore");
  }

  public void setPercent(double percent){
    rightTalonFX.set(percent);
    leftTalonFX.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
