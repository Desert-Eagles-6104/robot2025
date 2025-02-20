// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class AlgaeArmSubsystem extends ServoSubsystemTalon {
  /** Creates a new AlgaeArmSubsystem. */
  public AlgaeArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
  
  @Override
  public double toRotations(double units){
    return super.toRotations(units);
  }

  @Override
  public double fromRotations(double rotations){
    return super.fromRotations(rotations);
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

  public void smartSetPosition(double position){
    if(position < getPosition()){
      super.setMotionMagicPosition(position);
    }
    else{
      super.setPosition(position);
    }
}

  @Override
  public void setPrecentOutput(double precent){
    super.setPrecentOutput(precent);
  }

  @Override
  public double getPosition(){
    return super.getPosition();
  }

  @Override
  public double getVelocity(){
    return super.getVelocity();
  }

  @Override
  public double getMotorCurrent(){
    return super.getMotorCurrent();
  }

  @Override
  public double getClosedLoopError(){
    return super.getClosedLoopError();
  }

  @Override
  public void disableMotors(){
    super.disableMotors();
  }
}
