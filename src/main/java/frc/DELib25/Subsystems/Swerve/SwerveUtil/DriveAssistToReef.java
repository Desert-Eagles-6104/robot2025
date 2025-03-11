package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.ReefUtill;
import frc.robot.Robot;

public class DriveAssistToReef {    
  private double m_kpSide = 3.0;
  private double m_kpForward = 2.5;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;

    public DriveAssistToReef(){
        m_filterSide = LinearFilter.movingAverage(4);
        m_filterForward = LinearFilter.movingAverage(4);
    }

        public ChassisSpeeds update(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Boolean isLeft, Boolean isRight){
            ChassisSpeeds toReturn = new ChassisSpeeds();
            if(VisionSubsystem.getTv() && (isLeft || isRight)){
                Translation2d leftError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointLeft());
                Translation2d RightError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointRight());
                Translation2d finalPoint;
                if(isRight){
                    finalPoint = RightError;
                }
                else{
                    finalPoint = leftError;
                }
                //Shorter way To if else
                //finalPoint = isRight ? RightError : leftError;
        

                if(Robot.s_Alliance == Alliance.Red){
                    toReturn = ChassisSpeeds.fromRobotRelativeSpeeds(-m_filterForward.calculate(-finalPoint.getX())*m_kpForward, -m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0, robotHeading);
                }
                else{
                    toReturn = ChassisSpeeds.fromRobotRelativeSpeeds(m_filterForward.calculate(-finalPoint.getX())*m_kpForward, m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0, robotHeading);
                }
            }
            chassisSpeeds.vxMetersPerSecond = toReturn.vxMetersPerSecond + chassisSpeeds.vxMetersPerSecond;
            chassisSpeeds.vyMetersPerSecond = toReturn.vyMetersPerSecond + chassisSpeeds.vyMetersPerSecond;
            return chassisSpeeds;
        }
    }
