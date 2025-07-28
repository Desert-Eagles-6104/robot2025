package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.VisionSubsystemRobot2025;

public class DriveAssistAuto extends Command {
  private SwerveSubsystem swerveSubsystem;
  private double kpSide = 0.05;
  private double kpForward = 0.1;
  private LinearFilter filterSide;
  private LinearFilter filterForward;
  private ChassisSpeeds chassisSpeeds;
  private Translation2d centerOfRobot = new Translation2d();
  private StableBoolean dontSeesNoteForTime;
  private PoseEstimatorSubsystem poseEstimator;
  private VisionSubsystemRobot2025 vision;

  public DriveAssistAuto(SwerveSubsystem swerveSubsystem,VisionSubsystemRobot2025 vision, PoseEstimatorSubsystem poseEstimator){
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    this.filterSide = LinearFilter.movingAverage(4);
    this.filterForward = LinearFilter.movingAverage(4);
    chassisSpeeds = new ChassisSpeeds();
    dontSeesNoteForTime = new StableBoolean(0.5);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(this.vision.getTvNote()){
      if(Robot.s_Alliance == Alliance.Red){
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(-this.filterForward.calculate(-this.vision.getTyNote())*this.kpForward, -this.filterSide.calculate(-this.vision.getTxNote())*this.kpSide, 0, this.poseEstimator.getHeading());
      }
      else{
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.filterForward.calculate(-this.vision.getTyNote())*this.kpForward, this.filterSide.calculate(-this.vision.getTxNote())*this.kpSide, 0, this.poseEstimator.getHeading());
      }
    }
    else if(dontSeesNoteForTime.update(!this.vision.getTvNote())){
      chassisSpeeds.vxMetersPerSecond = 0;
      chassisSpeeds.vyMetersPerSecond = 0;
      chassisSpeeds.omegaRadiansPerSecond = 0;
    }
    this.swerveSubsystem.drive(chassisSpeeds, false, true, centerOfRobot);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
