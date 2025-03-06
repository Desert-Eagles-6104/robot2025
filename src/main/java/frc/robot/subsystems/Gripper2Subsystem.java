// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class Gripper2Subsystem extends SubsystemBase {
  TalonFX Gripper;
 private BeamBreak m_beamBreak;
 private boolean m_hasGamePiece;

  DutyCycle dutyCycleRequest;
  /** Creates a new Gripper2Subsystem. */
  public Gripper2Subsystem() {
    Gripper = new TalonFX(2);
   m_beamBreak = new BeamBreak(Constants.Gripper.beamBreakPort);
  }

  public boolean HasGamePiece(){
    return m_hasGamePiece;
  }

  @Override
  public void periodic() {
   m_beamBreak.update();
    m_hasGamePiece = m_beamBreak.get();
  }

  public void setPercent(double Percent){
  Gripper.set(Percent);
  }

  public void disableMotors(){
    Gripper.disable();
  }
}
