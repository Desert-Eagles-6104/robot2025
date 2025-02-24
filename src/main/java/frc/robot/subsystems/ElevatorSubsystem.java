// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ElevatorSubsystem extends ServoSubsystemTalon {
  /** Creates a new Elevator. */
  public ElevatorSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);  }

   public void periodic() {
    super.periodic();
  }

  @Override
  public void setPosition(double position){
    if(Math.abs(super.getClosedLoopError()) <2 ){
      super.setPosition(position);
    }
    else{
      super.setMotionMagicPosition(position);
    }
  }

 
}
