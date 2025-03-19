// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.util.ArrayBuilders.BooleanBuilder;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class Gripper2Subsystem extends SubsystemBase {
  
        TalonFX Gripper;
        public BeamBreak m_beamBreak2;
        private static boolean m_hasGamePiece;
        private Boolean see;
       
         DutyCycle dutyCycleRequest;
         /** Creates a new Gripper2Subsystem. */
         public Gripper2Subsystem() {
           Gripper = new TalonFX(2);
         m_beamBreak2 = new BeamBreak(Constants.Gripper.beamBreakPort);
         }
    
      public static boolean HasGamePiece(){
        return m_hasGamePiece;
  }

  @Override
  public void periodic() {
    m_beamBreak2.update();
    m_hasGamePiece = m_beamBreak2.get();
    SmartDashboard.putBoolean("BeamBreak", m_hasGamePiece);
  }

  public void setPercent(double Percent){
  Gripper.set(Percent);
  }

  public void disableMotors(){
    Gripper.disable();
  }
}
