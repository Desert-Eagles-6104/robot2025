package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.ReefUtill;
import frc.robot.Robot;

public class DriveAssistToReef {    
  private SwerveSubsystem m_swerveSubsystem;
  private double m_kpSide = 0.05;
  private double m_kpForward = 0.1;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;
  private ChassisSpeeds chassisSpeeds;
  private Translation2d centerOfRobot = new Translation2d();
  private StableBoolean dontSeesAprilTagForTime;

    public DriveAssistToReef(SwerveSubsystem swerveSubsystem){
        m_swerveSubsystem = swerveSubsystem;
        m_filterSide = LinearFilter.movingAverage(4);
        m_filterForward = LinearFilter.movingAverage(4);
        chassisSpeeds = new ChassisSpeeds();
        dontSeesAprilTagForTime = new StableBoolean(0.5);
    }

    public ChassisSpeeds update(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Boolean isRight){
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
            return chassisSpeeds;
        }
    }
