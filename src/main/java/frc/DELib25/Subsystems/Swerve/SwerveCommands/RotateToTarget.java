// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.HeadingController;

public class RotateToTarget extends Command {
  private SwerveSubsystem swerve;
  private HeadingController m_headingController;
  private Rotation2d target = new Rotation2d();
  private StableBoolean isFinish;
  private PoseEstimatorSubsystem poseEstimator;
  /** Creates a new RotateToTarget. */
  public RotateToTarget(SwerveSubsystem swerve,PoseEstimatorSubsystem poseEstimator) {
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    m_headingController = new HeadingController(0.2, 0, 0);
    isFinish = new StableBoolean(0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //target = PoseEstimatorSubsystem.getAngleToReef().unaryMinus();
    m_headingController.setSetpoint(target);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,m_headingController.update(this.poseEstimator.getHeading()));
    this.swerve.drive(chassisSpeeds, true, true, new Translation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinish.update(Math.abs(target.getDegrees() - this.poseEstimator.getHeading().getDegrees()) < 2);
  }
}
