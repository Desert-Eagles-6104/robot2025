// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.Robot;

public class DriveAssistAuto extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_kpSide = 0.05;
  private double m_kpForward = 0.1;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;
  private ChassisSpeeds chassisSpeeds;
  private Translation2d centerOfRobot = new Translation2d();
  private StableBoolean dontSeesNoteForTime;

  public DriveAssistAuto(SwerveSubsystem swerveSubsystem){
    m_swerveSubsystem = swerveSubsystem;
    m_filterSide = LinearFilter.movingAverage(4);
    m_filterForward = LinearFilter.movingAverage(4);
    chassisSpeeds = new ChassisSpeeds();
    dontSeesNoteForTime = new StableBoolean(0.5);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(VisionSubsystem.getTvNote()){
      if(Robot.s_Alliance == Alliance.Red){
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(-m_filterForward.calculate(-VisionSubsystem.getTyNote())*m_kpForward, -m_filterSide.calculate(-VisionSubsystem.getTxNote())*m_kpSide, 0, m_swerveSubsystem.getHeading());
      }
      else{
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_filterForward.calculate(-VisionSubsystem.getTyNote())*m_kpForward, m_filterSide.calculate(-VisionSubsystem.getTxNote())*m_kpSide, 0, m_swerveSubsystem.getHeading());
      }
    }
    else if(dontSeesNoteForTime.update(!VisionSubsystem.getTvNote())){
      chassisSpeeds.vxMetersPerSecond = 0;
      chassisSpeeds.vyMetersPerSecond = 0;
      chassisSpeeds.omegaRadiansPerSecond = 0;
    }
    m_swerveSubsystem.drive(chassisSpeeds, false, true, centerOfRobot);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
