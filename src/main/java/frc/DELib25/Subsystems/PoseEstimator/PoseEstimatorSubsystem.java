// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Sensors.Pigeon;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase{
  /** Creates a new PoseEstimator. */
  private static SwerveSubsystem m_swerve;
  private static Pigeon m_gyro;
  private static LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private static boolean first = true;
  private static StableBoolean tvStableBoolean;
  public PoseEstimatorSubsystem(SwerveSubsystem swerve) {
    m_swerve = swerve;
    m_gyro = Pigeon.getInstance();
    tvStableBoolean = new StableBoolean(0.15);
  }

  @Override
  public void periodic() {
    if(!first){
      updateVisionOdometry();
    }
    else{ 
      first = false;
    }
  }

  private static void updateVisionOdometry(){
    if(!first){ 
      boolean rejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight", getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      limelightMesermentMT2 = VisionSubsystem.getEstimatedRobotPose();
      if(Math.abs(m_gyro.getRateStatusSignalWorld().getValueAsDouble()) > 360 || limelightMesermentMT2.pose == null){
        rejectUpdate = true;
      }
      if(!rejectUpdate && tvStableBoolean.get(VisionSubsystem.getTv()) && limelightMesermentMT2.pose != null){
        m_swerve.addVisionMeasurement(limelightMesermentMT2.pose, limelightMesermentMT2.timestampSeconds);
      }
    }
    else{
      first = false;
    }
  }

  public static Pose2d getRobotPose(){
    return m_swerve.getPose();
  }

  public static void resetPosition(Pose2d pose){
    m_swerve.resetOdometry(pose);
  }

  public static void resetPositionFromCamera(){
    if(limelightMesermentMT2.pose != null){
      resetPosition(limelightMesermentMT2.pose);
    }
  }
  
  public static void zeroHeading(){
    m_swerve.zeroHeading();
  }

    public static Rotation2d getHeading() {
    return getRobotPose().getRotation();
  }

  public static Pose2d getInterpolatedPose(double latencySeconds){
    return m_swerve.getInterpolatedPose(latencySeconds);
  }

   /**
  @param value clamped value
  @param min min value
  @param max max value
  @return sets a range for the value if its between the max and min points
  */
  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }


}
