// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemConfiguration;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class ElevatorSubsystem extends MotorSubsystemTalon {
  private DigitalInput elevatorMagnet;
  /** Creates a new Elevator. */
  public ElevatorSubsystem(MotorSubsystemConfiguration configuration) {
    super(configuration);  
    elevatorMagnet = new DigitalInput(1);
  }
   public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean("Elevator Magnet", this.getMagnetState());
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

  public boolean getMagnetState(){
    return this.elevatorMagnet.get();
  }
 
}
