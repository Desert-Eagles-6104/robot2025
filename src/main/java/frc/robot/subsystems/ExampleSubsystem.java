// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    TalonFX kraken1;
    TalonFX kraken2;
      private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

  public ExampleSubsystem() {
    kraken1 = new TalonFX(1);
    kraken2 = new TalonFX(2);
    
  }

  public void set(double speed){

    kraken1.setControl(m_dutyCycleRequest.withOutput(speed));
    kraken2.setControl(m_dutyCycleRequest.withOutput(-speed));
  }

}