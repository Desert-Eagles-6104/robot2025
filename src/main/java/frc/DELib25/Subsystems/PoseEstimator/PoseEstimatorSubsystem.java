// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Sensors.Pigeon;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Subsystems.Vision.VisionUtil.LimelightHelpers;
import frc.robot.Robot;

public class PoseEstimatorSubsystem extends SubsystemBase{
  /** Creates a new PoseEstimator. */
  private static SwerveSubsystem m_swerve;
  private static Pigeon m_gyro;
  private static LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private static double speakerHighetFromRobot = 2.049-0.136;
  private static double odometryToArmDistance = 0.13784;
  private static boolean first = true;
  private static StableBoolean tvStableBoolean;
  private static Translation2d blueSpeaker = new Translation2d(0.0, 5.55);
  private static Translation2d redSpeaker = new Translation2d(16.52, 5.55);
  private static double blueWing = 5.85;
  private static double redWing = 10.7;
  private static double centerLine = 4.1;
  private static Translation2d DeliveryBlueSpeaker = new Translation2d(0.95, 7.0);
  private static Translation2d DeliveryRedSpeaker = new Translation2d(15.75, 7.0);
  private static Translation2d deliveryMiddleBlue = new Translation2d(7.0, 7.35);
  private static Translation2d deliveryMiddleRed = new Translation2d(9.5, 7.35);

  public PoseEstimatorSubsystem(SwerveSubsystem swerve) {
    m_swerve = swerve;
    m_gyro = Pigeon.getInstance();
    tvStableBoolean = new StableBoolean(0.15);
  }

  @Override
  public void periodic() {
    if(!first){
      updateVisionOdometry();
      SmartDashboard.putNumber("distance from speaker", getDistanceToSpeaker());
      SmartDashboard.putNumber("angleSpeaker", getAngleToSpeaker().getDegrees());
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
      if(Math.abs(m_gyro.getRateStatusSignalWorld().getValueAsDouble()) > 360 && getRobotPose().getX() < 5 || limelightMesermentMT2.pose == null){
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

  public static boolean isCentered(){
    return Math.abs(getHeading().getDegrees() - getAngleToSpeaker().getDegrees()) < 1.5; //TODO: cheak
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

    public static Rotation2d getAngleToDelivery(){
      if(inEnenmyWing() && Robot.s_Alliance == Alliance.Red){
        return Rotation2d.fromRadians(-Math.atan((deliveryMiddleRed.getY() - getRobotPose().getY())/(deliveryMiddleRed.getX() -getRobotPose().getX()))).rotateBy(Rotation2d.fromDegrees(180));
      }
      else if(inEnenmyWing() && Robot.s_Alliance == Alliance.Blue){
        return Rotation2d.fromRadians(-Math.atan((deliveryMiddleBlue.getY() - getRobotPose().getY())/(deliveryMiddleBlue.getX() -getRobotPose().getX())));
      }
      else if(notInAnyWing() && Robot.s_Alliance == Alliance.Red){
        return Rotation2d.fromRadians(-Math.atan((DeliveryRedSpeaker.getY() - getRobotPose().getY())/(DeliveryRedSpeaker.getX() -getRobotPose().getX()))).rotateBy(Rotation2d.fromDegrees(180));
      }
      else if(notInAnyWing() && Robot.s_Alliance == Alliance.Blue){
        return Rotation2d.fromRadians(-Math.atan((DeliveryBlueSpeaker.getY() - getRobotPose().getY())/(DeliveryBlueSpeaker.getX() -getRobotPose().getX())));
      }
      else{
        return Rotation2d.fromDegrees(0);
      }
  }

  public static double getDistanceToSpeaker(){
    if(Robot.s_Alliance == Alliance.Red){
    return getRobotPose().getTranslation().getDistance(redSpeaker);
    }
    return getRobotPose().getTranslation().getDistance(blueSpeaker);
  }

  public static Rotation2d getAngleToSpeaker(){
    if(Robot.s_Alliance == Alliance.Red){
      return Rotation2d.fromRadians(-Math.atan((redSpeaker.getY() - getRobotPose().getY())/(redSpeaker.getX() -getRobotPose().getX()))).rotateBy(Rotation2d.fromDegrees(180));
    }
    return Rotation2d.fromRadians(-Math.atan((blueSpeaker.getY() - getRobotPose().getY())/(blueSpeaker.getX() -getRobotPose().getX())));
  }

  public static double getArmAngleToBlueSpeaker(){
    double distanceToSpeaker = getDistanceToSpeaker()+odometryToArmDistance;
    return clamp(Math.toDegrees(Math.atan((speakerHighetFromRobot)/(distanceToSpeaker)))+7.9  , 10, 100);
  } 

  public static boolean inMyWing(){
    if(Robot.s_Alliance == Alliance.Red){
      return getRobotPose().getX() >= redWing;
    }
    else{
      return getRobotPose().getX() <= blueWing;
    }
  }

  public static boolean inEnenmyWing(){
    if(Robot.s_Alliance == Alliance.Red){
      return getRobotPose().getX() <= blueWing;
    }
    else{
      return getRobotPose().getX() >= redWing;
    }
  }

  public static boolean isAtFeederSide(){
    return getRobotPose().getY() <= centerLine;
  }

  public static boolean notInAnyWing(){
    return !inEnenmyWing() && !inMyWing();
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
