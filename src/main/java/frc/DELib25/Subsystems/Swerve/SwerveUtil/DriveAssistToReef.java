package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib25.Subsystems.Vision.VisionSubsystem;
import frc.robot.ReefUtill;
import frc.robot.Robot;

public class DriveAssistToReef {    
  private double m_kpSide = 3.0;
  private double m_kpForward = 3.0;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;

    public DriveAssistToReef(){
        m_filterSide = LinearFilter.movingAverage(4);
        m_filterForward = LinearFilter.movingAverage(4);
    }

        public ChassisSpeeds update(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Boolean isLeft, Boolean isRight){
            ChassisSpeeds toReturn = new ChassisSpeeds();
            if((isLeft || isRight)){ // removed getTV 
                Translation2d leftError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointLeft());
                Translation2d RightError = PoseEstimatorSubsystem.getRobotPose().getTranslation().minus(ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointRight());
                Translation2d finalPoint;
                if(isRight){
                    finalPoint = RightError;
                    SmartDashboard.putNumber("FinalPoint getx", ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointLeft().getX());
                    SmartDashboard.putNumber("FinalPoint gety",  ReefUtill.getReefFacePoint(ReefUtill.getFaceFromVision()).getPointLeft().getY());
                    SmartDashboard.putNumber("finalPointErrorX", finalPoint.getX());
                    SmartDashboard.putNumber("finalPointErrorY",finalPoint.getY());

                }
                else{
                    finalPoint = leftError;
                }
                //Shorter way To if else
                //finalPoint = isRight ? RightError : leftError;
        

                if(Robot.s_Alliance == Alliance.Red){
                    toReturn = ChassisSpeeds.fromFieldRelativeSpeeds(m_filterForward.calculate(-finalPoint.getX())*m_kpForward, m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0,Rotation2d.fromDegrees(getAngleToreef(robotHeading)));
                }
                else{
                    toReturn = ChassisSpeeds.fromFieldRelativeSpeeds(m_filterForward.calculate(-finalPoint.getX())*m_kpForward, m_filterSide.calculate(-finalPoint.getY())*m_kpSide, 0,Rotation2d.fromDegrees(getAngleToreef(robotHeading)));
                }
            }
            chassisSpeeds.vxMetersPerSecond = toReturn.vxMetersPerSecond + chassisSpeeds.vxMetersPerSecond;
            chassisSpeeds.vyMetersPerSecond = toReturn.vyMetersPerSecond + chassisSpeeds.vyMetersPerSecond;
            return chassisSpeeds;
        }

        private double getAngleToreef(Rotation2d wantedAngle) {
            try {
                double currentHeading = PoseEstimatorSubsystem.getHeading().getDegrees();
                double angleDiff = currentHeading - wantedAngle.getDegrees();
        
                if (angleDiff > 180) {
                    angleDiff -= 360;
                } else if (angleDiff < -180) {
                    angleDiff += 360;
                }
        
                return angleDiff;
            } catch (Exception e) {
                // Handle the exception appropriately, for example, print the error message
                System.err.println("An error occurred while calculating the angle: " + e.getMessage());
                return 0.0;  // Return a default value in case of an error
            }
        }
        
            
                
            
    }
