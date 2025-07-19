// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.PoseEstimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Intepulation.InterpolatingDouble;
import frc.DELib25.Intepulation.InterpolatingTreeMap;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

/** Creates a new PoseEstimator. */
public class PoseEstimatorSubsystem extends SubsystemBase {

  private SwerveSubsystem swerve;
  private LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private boolean first;
  private StableBoolean tvStableBoolean;
  private SwerveDrivePoseEstimator odometry;
  private Field2d field;
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> pastPoses;

  public PoseEstimatorSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;

    this.odometry = new SwerveDrivePoseEstimator(
      this.swerve.getKinematics(),
      Rotation2d.fromDegrees(0),
      this.swerve.getModulesPositions(), new Pose2d(),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.3, 0.3, 9999999)
    );

    SmartDashboard.putData("Field", this.field);
    
    this.first = true;
    this.tvStableBoolean = new StableBoolean(0.15);
    this.field = new Field2d();

    this.pastPoses = new InterpolatingTreeMap<>(51); // Represents the max pose history size
  }

  @Override
  public void periodic() {

    if (!this.first) {
      updateVisionOdometry();
    } else {
      this.first = false;
    }
    
    Pose2d currentPose = this.updateOdometry();
    this.pastPoses.put(new InterpolatingDouble(Timer.getFPGATimestamp()), currentPose);
    SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
    SmartDashboard.putNumber("robotX", this.getPose().getX());
    SmartDashboard.putNumber("robotY ", this.getPose().getY());
    SmartDashboard.putNumber("robotorientation", this.getPose().getRotation().getDegrees());

    this.updateOdometry();
    this.field.setRobotPose(this.odometry.getEstimatedPosition());
  }

  public void updateVisionOdometry() {
    if (!this.first) {
      boolean rejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight-april", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      this.limelightMesermentMT2 = VisionSubsystem.getEstimatedRobotPose();
      if (Math.abs(this.swerve.getGyro().getRateStatusSignalWorld().getValueAsDouble()) > 360
          || this.limelightMesermentMT2.pose == null) {
        rejectUpdate = true;
      }
      if (!rejectUpdate && this.tvStableBoolean.update(VisionSubsystem.getTv())
          && this.limelightMesermentMT2.pose != null) {
        this.addVisionMeasurement(this.limelightMesermentMT2.pose, this.limelightMesermentMT2.timestampSeconds);
      }
    } else {
      this.first = false;
    }
  }

  public void resetOdometryToPose(Pose2d pose) {
    this.odometry.resetPosition(this.swerve.getGyro().getYaw(), this.swerve.getModulesPositions(), pose);
  }

  public Pose2d updateOdometry() {
    return this.odometry.update(this.swerve.getGyro().getYaw(), this.swerve.getModulesPositions());
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    this.odometry.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(0.7, 0.7, 9999999));
  }

  public Pose2d getPose() {
    return this.odometry.getEstimatedPosition();
  }
  
  public Rotation2d getHeading() {
    return this.getPose().getRotation();
  }

  public void resetPositionFromCamera() {
    if (this.limelightMesermentMT2.pose != null) {
      this.resetOdometryToPose(this.limelightMesermentMT2.pose);
    }
  }
  
  public Pose2d getInterpolatedPose(double latencySeconds) {
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;
    return this.pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
  }
}
