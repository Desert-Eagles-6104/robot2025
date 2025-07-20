// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.MotorSubsystems.Base.Motor.ServoSubsystemTalon;

public class IntakeArmSubsystem extends ServoSubsystemTalon {
  /** Creates a new AlgaeArmSubsystem. */
  public IntakeArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void setMotionMagicPosition(double position) {
    if(position < getPosition()){
      super.setMotionMagicPosition(position);
    }
    else{
      super.setPosition(position);
    }
  }

  @Override
  public void setPosition(double position){
    if(position < getPosition()){
      super.setMotionMagicPosition(position);
    }
    else{
      super.setPosition(position);
    }
  }
}
