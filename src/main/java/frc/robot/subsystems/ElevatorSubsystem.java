// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ElevatorSubsystem extends ServoSubsystemTalon {
  private BeamBreak m_ElevatorMagnet;
  private boolean magnetState = false;
  /** Creates a new Elevator. */
  public ElevatorSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    m_ElevatorMagnet = new BeamBreak(2);
  }

   public void periodic() {
    super.periodic();
    magnetState = magnetUpdate();
    // SmartDashboard.putBoolean("magnetcontact", magnetState);
    // SmartDashboard.putNumber("ElevatorSetpoint", setpoint);
  }

  public boolean magnetUpdate(){
    m_ElevatorMagnet.update();
    return m_ElevatorMagnet.get();
  }
  
  public boolean getMagnetState(){
    return magnetState;
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
    super.setMotionMagicPosition(position);
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

  @Override
  public void setPrecentOutput(double percent){
    super.setPrecentOutput(percent);
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

  @Override 
  public void runCharacterization(Voltage volts){
    super.runCharacterization(volts);
  }
}
