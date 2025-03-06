// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib25.BooleanUtil.LatchedBolean;
import frc.DELib25.BooleanUtil.ToggleBoolean;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.DriveAssistToReef;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.HeadingController;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveDriveHelper;
import frc.DELib25.Subsystems.Swerve.SwerveUtil.SwerveDriveHelper.DriveMode;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.DELib25.Util.Elastic;
import frc.robot.Constants;
import frc.robot.ReefUtill;
import frc.robot.Robot;

public class TeleopDrive extends Command {
 private  SwerveSubsystem m_swerve;
 private CommandPS5Controller m_joystick;
 private HeadingController m_headingController;
 private DriveAssistToReef m_driveAssistToReefController;
 private BooleanSupplier m_lowPower;
 private BooleanSupplier m_fieldRelative;
 private BooleanSupplier m_shouldResetYaw;
 private BooleanSupplier m_useVision;
 private LatchedBolean m_useVisionLatch;
 private ToggleBoolean m_fieldRelativeToggle;
 private Translation2d m_centerOfRotation;
 private BooleanSupplier m_isRight;
 private BooleanSupplier m_isLeft;

  public TeleopDrive(SwerveSubsystem swerve ,CommandPS5Controller joystick, BooleanSupplier lowPower, BooleanSupplier fieldRelative, BooleanSupplier resetYaw, BooleanSupplier useVision, BooleanSupplier isRight, BooleanSupplier isLeft) {
    m_swerve = swerve;
    m_joystick = joystick;
    m_headingController = new HeadingController(new PIDContainer(0.06, 0, 0, "stablize"), new PIDContainer(0.09, 0, 0, "snap"), new PIDContainer(0.2, 0.0, 0.0, "vision"), new PIDContainer(0.2, 0.00001, 0.0, "visionLowError")); //vision note: p=0.1  visionNoteLowError: p =0.1 i = 0.00001
    m_driveAssistToReefController = new DriveAssistToReef();
    m_lowPower = lowPower;
    m_fieldRelative = fieldRelative;
    m_shouldResetYaw = resetYaw;
    m_useVision = useVision;
    m_useVisionLatch = new LatchedBolean();
    m_fieldRelativeToggle = new ToggleBoolean();
    m_centerOfRotation = new Translation2d();
    m_isRight = isRight;
    m_isLeft = isLeft;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.1),
      MathUtil.applyDeadband(-m_joystick.getLeftX(), 0.1),
      MathUtil.applyDeadband(-m_joystick.getRightX(), 0.1));
      chassisSpeeds = SwerveDriveHelper.updateChassisSpeeds(chassisSpeeds, m_lowPower, DriveMode.MadTown);
      chassisSpeeds = SwerveDriveHelper.joystickToRobotUnits(chassisSpeeds, Constants.Swerve.swerveConstants.maxSpeed, Constants.Swerve.swerveConstants.maxAngularVelocity);
      //heading controller
       m_useVisionLatch.update(m_useVision.getAsBoolean());
      if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.4){
        m_useVisionLatch.reset();
      }
      
      setVisionTargetlocalization(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getRobotAngleToFace().getDegrees());
      chassisSpeeds = m_headingController.calculateOmegaSpeed2(!Robot.s_isAuto ,shouldResetAngle(m_shouldResetYaw), true, chassisSpeeds, PoseEstimatorSubsystem.getHeading(), PoseEstimatorSubsystem.getRobotPose().getRotation(), m_swerve.getRobotRelativeVelocity());

      chassisSpeeds = m_driveAssistToReefController.update(chassisSpeeds, PoseEstimatorSubsystem.getHeading(), m_isLeft.getAsBoolean() ,m_isRight.getAsBoolean());

      m_swerve.drive(chassisSpeeds, true, m_fieldRelativeToggle.update(!m_fieldRelative.getAsBoolean()), m_centerOfRotation);

      SmartDashboard.putNumber("omegaRadiansPerSecond", chassisSpeeds.omegaRadiansPerSecond);
      SmartDashboard.putNumber("vxMetersPerSecond", chassisSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("vyMetersPerSecond", chassisSpeeds.vyMetersPerSecond);


      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean shouldResetAngle(BooleanSupplier shouldResetYaw){
    if(shouldResetYaw.getAsBoolean()){
      m_swerve.zeroHeading();
      return true;
    }
    return false;
  }

  /**
   * @param hasTarget camera sees target 
   * @param errorFromTarget error from target in degrees
   * @param latency camera total latency
   */
  private void setVisionTarget(boolean hasTarget, double errorFromTarget, double latency){
    if(m_useVisionLatch.get()){
      double errorDegrees = hasTarget ? -errorFromTarget : 0;
      Rotation2d target = PoseEstimatorSubsystem.getInterpolatedPose(VisionSubsystem.getTotalLatency()).getRotation().plus(Rotation2d.fromDegrees(errorDegrees));
      m_headingController.setSetpoint(target);
    }
  }

    /**
   * @param hasTarget camera sees target 
   * @param errorFromTarget error from target in degrees
   */
  private void setVisionTargetlocalization(double errorFromTarget){
    if(m_useVisionLatch.get()){
      double errorDegrees = -errorFromTarget;
      Rotation2d target = Rotation2d.fromDegrees(errorDegrees);
      m_headingController.setSetpoint(target);
    }
  } 
}