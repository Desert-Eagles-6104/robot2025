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
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.ReefUtill;
import frc.robot.Robot;

public class ReefAssist extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_kpSide = 3.0;
  private double m_kpForward = 3.0;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;
  private ChassisSpeeds chassisSpeeds;
  private Translation2d centerOfRobot = new Translation2d();
  private StableBoolean dontSeesAprilTagForTime;
  private boolean isRight = false;

  public ReefAssist(SwerveSubsystem swerveSubsystem){
    m_swerveSubsystem = swerveSubsystem;
    m_filterSide = LinearFilter.movingAverage(4);
    m_filterForward = LinearFilter.movingAverage(4);
    chassisSpeeds = new ChassisSpeeds();
    dontSeesAprilTagForTime = new StableBoolean(0.5);
    
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(VisionSubsystem.getTv()){
      Translation2d leftError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointLeft());
      Translation2d RightError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointRight());
      Translation2d finalPoint;

      if(isRight){
        finalPoint = RightError;
      }
      else{
        finalPoint = leftError;
      }
    
      if(Robot.s_Alliance == Alliance.Red){
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(-m_filterForward.calculate(-finalPoint.getX())*m_kpForward, -m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0, m_swerveSubsystem.getHeading());
      }
      else{
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_filterForward.calculate(-finalPoint.getX())*m_kpForward, m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0, m_swerveSubsystem.getHeading());
      }
    }

    else if(dontSeesAprilTagForTime.get(!VisionSubsystem.getTv())){
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
