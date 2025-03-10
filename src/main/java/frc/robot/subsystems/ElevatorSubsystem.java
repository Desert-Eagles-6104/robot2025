// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ElevatorSubsystem extends ServoSubsystemTalon {
  private BeamBreak m_ElevatorMagnet;
  private boolean magnetState = false;
  /** Creates a new Elevator. */
  public ElevatorSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);  
    m_ElevatorMagnet = new BeamBreak(1);
  }
   public void periodic() {
    super.periodic();
    magnetState = magnetUpdate();
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

  public boolean magnetUpdate(){
    m_ElevatorMagnet.update();
    return m_ElevatorMagnet.get();
  }

  public boolean getMagnetState(){
    return magnetState;
  }


 
}
