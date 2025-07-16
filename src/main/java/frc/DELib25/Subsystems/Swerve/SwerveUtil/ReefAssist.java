// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.Robot;

public class ReefAssist extends Command {
  private SwerveSubsystem swerveSubsystem;
  private double kpSide = 2.0;
  private double kpForward = 2.7;
  private LinearFilter filterSide = LinearFilter.movingAverage(4);
  private LinearFilter filterForward = LinearFilter.movingAverage(4);
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private Translation2d centerOfRobot = new Translation2d();
  private StableBoolean dontSeesAprilTagForTime = new StableBoolean(0.5);
  private boolean isRight = false;
  private PoseEstimatorSubsystem poseEstimator;
  Translation2d leftError;
  Translation2d RightError;
  Translation2d finalPoint;

  public ReefAssist(SwerveSubsystem swerveSubsystem , PoseEstimatorSubsystem poseEstimator, BooleanSupplier right , BooleanSupplier left){
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    isRight = right.getAsBoolean();
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
      if(isRight){
        finalPoint = RightError;
      }
      else{
        finalPoint = leftError;
      }
    
      if(Robot.s_Alliance == Alliance.Red){
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(-this.filterForward.calculate(-finalPoint.getX())*this.kpForward, -this.filterSide.calculate(-finalPoint.getY())*this.kpSide, 0, this.poseEstimator.getHeading());
      }
      else{
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.filterForward.calculate(-finalPoint.getX())*this.kpForward, this.filterSide.calculate(-finalPoint.getY())*this.kpSide, 0, this.poseEstimator.getHeading());
      }

     if(dontSeesAprilTagForTime.update(!VisionSubsystem.getTv())){
      chassisSpeeds.vxMetersPerSecond = 0;
      chassisSpeeds.vyMetersPerSecond = 0;
      chassisSpeeds.omegaRadiansPerSecond = 0;
    }

    this.swerveSubsystem.drive(chassisSpeeds, true, true, centerOfRobot);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  public double SetID(){
    double id = VisionSubsystem.getID();//TODO add && to ifs
    return id;
  }


  }